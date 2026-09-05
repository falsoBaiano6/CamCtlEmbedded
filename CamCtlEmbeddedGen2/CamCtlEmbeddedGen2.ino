/*
 * Arduino UNO R4 WiFi — USB-Serial to LANC + Pan/Tilt Interface v1.0
 *
 * Pan/Tilt outputs: default HIGH-Z (INPUT), driven LOW when actuated.
 * LANC: bidirectional split into CMD_OUT (output) and SIG_IN (input).
 * Only one LANC port active at a time.
 * - LANC command output is interrupt-driven using a hardware timer (AGT0).
 * - LANC SIG_IN pins are monitored via pin-change / external interrupts.
 * Serial handshake uses '<' '>' framing; host announces with '!'.
 */
 
/* In the standard 8-byte LANC packet frame, an external controller (acting as a 
slave/remote relative to the camera's master timing) injects command data 
exclusively into Byte 0 and Byte 1.
Packet Structure and Injection Slots
Byte 0: First command slot (e.g., zoom/focus or record start/stop commands).
Byte 1: Second command slot (extended command or secondary parameters).
Bytes 2 & 3: Reserved for tuners or extra control/command extensions 
(usually left as 00 by standard controllers).
Bytes 4 through 7: Camera-to-controller VCR status bytes, counter data, and 
device status (driven strictly by the camera master).
How Injection Works
The camera (master) generates the clock and pulls the line low to signal the 
start bit for each of the 8 bytes in a frame.
The external device (slave/remote) listens for the start bit of Byte 0, waits 
for the precise bit timing, and pulls the open-collector line low to inject 
its command bits into Byte 0 and immediately after during Byte 1.
For the remaining bytes (Bytes 2–7), the external controller stops driving the 
line and only listens to the status information transmitted by the camera. */

#include "FspTimer.h"   // RA4M1 FSP timer wrapper bundled with UNO R4 core
#include "r_gpt.h"      // Renesas GPT header
#include <WiFiS3.h>
#include <WiFiUdp.h>
#include "Arduino.h"

// ─── Protocol constants ──────────────────────────────────────────────────────
#define HostListeningCode '!'
#define STX               '<'
#define ETX               '>'

#define rxAckSTX   "FE"
#define rxAckCh    "AA"
#define rxAckETX   "EF"
#define rxAckCID   '@'
#define rxAckCMD   '$'
#define rxAckCmplt '%'

#define cam1Id    '1'
#define cam2Id    '2'
#define cam3Id    '3'

// ─── Command codes ─────────────
#define CMD_PAN_LEFT    'L'
#define CMD_PAN_RIGHT   'R'
#define CMD_TILT_UP     'U'
#define CMD_TILT_DOWN   'D'
#define CMD_PAN_STOP    'S'   // release all pan/tilt for active camera
#define CMD_LANC        'Z' 

// ─── Pin assignments ─────────────────────────────────────────────────────────
// CAM1
#define CAM1_LANC_CMD_OUT   5
#define CAM1_LANC_SIG_IN   18
#define CAM1_PAN_LEFT      10
#define CAM1_PAN_RIGHT     11
#define CAM1_TILT_DOWN     12
#define CAM1_TILT_UP       13

// CAM2
#define CAM2_LANC_CMD_OUT   6
#define CAM2_LANC_SIG_IN   16
#define CAM2_PAN_LEFT      17
#define CAM2_PAN_RIGHT      7
#define CAM2_TILT_DOWN      8
#define CAM2_TILT_UP       15

// CAM3
#define CAM3_LANC_CMD_OUT   9
#define CAM3_LANC_SIG_IN   14
#define CAM3_PAN_LEFT       2
#define CAM3_PAN_RIGHT      3
#define CAM3_TILT_DOWN      4
#define CAM3_TILT_UP       19
#define NUM_CAMERAS         3

// LANC Wire color pin assignment:
// Tip -- White
// Ring -- Red
// Sleeve -- Black
// Note: Tip and Ring colors are juxtaposed on some teast leads

// LANC commands:
// Zoom In (Tele):
// Slowest: 28 00
// Medium: 28 04
// Fastest: 28 0E
// Zoom Out (Wide):
// Slowest: 28 10in
// Medium: 28 14
// Fastest: 28 1E

// LANC timing constants at 9600 baud (in microseconds)
const uint32_t BIT_TIME_US = 104;
const uint32_t HALF_BIT_TIME_US = 52;
const uint32_t SYNC_GAP_MIN_US = 5000; // 5ms sync preamble threshold

// ─────────────────────────────────────────────────────────────
// Per-Channel Lanc State
// ─────────────────────────────────────────────────────────────
struct LancChannel {
    uint8_t rxPin;
    uint8_t txPin;

    volatile uint32_t last_falling_edge;
    volatile bool     active;        // selected camera
    volatile bool     cmdPending;

    uint8_t txBuf[2];                // bytes 0–1 to inject
    uint8_t rxBuf[8];                // bytes 0–7 readback

    volatile uint8_t  state;

    // Status read scaffolding (bytes 2–7)
    volatile uint8_t statusByteIdx;  // 2..7
    volatile uint8_t statusBitIdx;   // 0..9 (start+8+stop)
};

LancChannel lanc[3];

// ─── Pin lookup tables (index: 0=CAM1, 1=CAM2, 2=CAM3) ───────────────────────
const uint8_t lancCmdPin[NUM_CAMERAS]  = { CAM1_LANC_CMD_OUT, CAM2_LANC_CMD_OUT, CAM3_LANC_CMD_OUT };
const uint8_t lancSigPin[NUM_CAMERAS]  = { CAM1_LANC_SIG_IN,  CAM2_LANC_SIG_IN,  CAM3_LANC_SIG_IN  };
const uint8_t panLeftPin[NUM_CAMERAS]  = { CAM1_PAN_LEFT,  CAM2_PAN_LEFT,  CAM3_PAN_LEFT  };
const uint8_t panRightPin[NUM_CAMERAS] = { CAM1_PAN_RIGHT, CAM2_PAN_RIGHT, CAM3_PAN_RIGHT };
const uint8_t tiltUpPin[NUM_CAMERAS]   = { CAM1_TILT_UP,   CAM2_TILT_UP,   CAM3_TILT_UP   };
const uint8_t tiltDownPin[NUM_CAMERAS] = { CAM1_TILT_DOWN, CAM2_TILT_DOWN, CAM3_TILT_DOWN };

const uint8_t allPanTiltPins[] = {
  CAM1_PAN_LEFT, CAM1_PAN_RIGHT, CAM1_TILT_UP, CAM1_TILT_DOWN,
  CAM2_PAN_LEFT, CAM2_PAN_RIGHT, CAM2_TILT_UP, CAM2_TILT_DOWN,
  CAM3_PAN_LEFT, CAM3_PAN_RIGHT, CAM3_TILT_UP, CAM3_TILT_DOWN
};

const uint8_t validCamValues[NUM_CAMERAS] = { cam1Id, cam2Id, cam3Id };
#define NUM_COMMANDS 6
const uint8_t validCmdValues[NUM_COMMANDS] = { CMD_PAN_LEFT, CMD_PAN_RIGHT, CMD_TILT_UP, CMD_TILT_DOWN, CMD_PAN_STOP, CMD_LANC }; 
volatile uint8_t lancActiveSigPin = CAM1_LANC_SIG_IN;

// ─────────────────────────────────────────────────────────────
// State Machine States
// ─────────────────────────────────────────────────────────────

// ─── Lanc State (interrupt-driven, one camera active) ──────────────────
enum LancState {
    SEARCHING_SYNC,
    WAITING_START,
    PROCESSING_BYTE,
    READING_STATUS
};
	
// ─── Host message frame state (tight handshake -- 1-2 characters, 1 Ack) ──────────────────

enum FrameState {
    IDLE,
    CID,
    CMD,
    DATA
};
	
	
// ─── LANC timing ──────────────────────────────────────────────────────────────
// LANC runs at 9600 baud → 1 bit = 104.167 µs.
// Timer fires every half-bit (52 µs) so we can sample at mid-bit as well.
// Each LANC frame = start bit + 8 data bits + stop bit = 10 half-bit pairs = 20 ticks.
// Two bytes per frame → 40 ticks total.

// ─── Application state ────────────────────────────────────────────────────────
volatile int8_t  activeCam      = -1;   // 0-based; -1 = none selected
volatile bool lancPacketComplete = false;
FrameState currentState = IDLE;
static bool validCamId = false;
static bool validCmd = false;

// ─── Bit-Bang control ───────────────────────────────────────────────────────────
volatile bool lancBitBangActive = false;
volatile uint32_t lancStartTime = 0;


// Pending command loaded by main loop

volatile bool    lancCmdReceived = false;   
volatile bool    lancCmdPending = false;


// variables tracking the LANC start timing state
volatile bool actionComplete = false;

// repeat counters for multi-frame operation
volatile uint8_t  lancFrameRepeatCount    = 1;  // how many frames to send
volatile uint8_t  lancFrameRepeatRemaining = 0; // countdown

FspTimer lancTimer;

// ─── Serial receive buffer ────────────────────────────────────────────────────
#define RX_DATA_BUF_SIZE 8
static char  rxDataBuf[RX_DATA_BUF_SIZE];
static uint8_t rxDataBufIdx  = 0;
static uint8_t rxDataLen = 0;

// ─────────────────────────────────────────────────────────────
// Select active channel (called by host command logic)
// ─────────────────────────────────────────────────────────────
void selectLancChannel(int idx) {
    activeCam = idx;   // 0..2 or -1 for inactive

    for (int i = 0; i < 3; i++) {
        lanc[i].active = (i == idx);
        lanc[i].state  = SEARCHING_SYNC;
        lanc[i].statusByteIdx = 2;
        lanc[i].statusBitIdx  = 0;
        digitalWrite(lanc[i].txPin, LOW);  // idle
    }
}

// ─────────────────────────────────────────────────────────────
// Queue LANC command (bytes 0–1)
// ─────────────────────────────────────────────────────────────
void queueLancCommand(uint8_t b0, uint8_t b1, uint8_t repeatFrames = 1) {
    if (activeCam < 0) return;

    lanc[activeCam].txBuf[0] = b0;
    lanc[activeCam].txBuf[1] = b1;
    lanc[activeCam].cmdPending = true;
		
    lancFrameRepeatCount     = repeatFrames;
    lancFrameRepeatRemaining = repeatFrames;
}

// Release all pan/tilt pins to HIGH-Z (INPUT, no pull-up)
void releaseAllPanTilt() {
  for (uint8_t i = 0; i < sizeof(allPanTiltPins); i++) {
    pinMode(allPanTiltPins[i], INPUT);
  }
}

// Release only the pan/tilt pins for one camera
void releasePanTilt(uint8_t camIdx) {
  pinMode(panLeftPin[camIdx],  INPUT);
  pinMode(panRightPin[camIdx], INPUT);
  pinMode(tiltUpPin[camIdx],   INPUT);
  pinMode(tiltDownPin[camIdx], INPUT);
}

// Drive one pan/tilt pin LOW (all others for that camera released first)
void actuatePanTilt(uint8_t camIdx, uint8_t pin) {
  releasePanTilt(camIdx);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

// Functions to check membership
bool isValidCamId(uint8_t target) {
  for (uint8_t i = 0; i < NUM_CAMERAS; i++) {
    if (validCamValues[i] == target) {
      return true; // Found a match
    }
  }
  return false; // Checked everything, no match
}

bool isValidCmd(uint8_t target) {
  for (uint8_t i = 0; i < NUM_COMMANDS; i++) {
    if (validCmdValues[i] == target) {
      return true; // Found a match
    }
  }
  return false; // Checked everything, no match
}

// ─── ISRs ─────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
// External Falling-Edge ISR (per channel)
// ─────────────────────────────────────────────────────────────
void lancTriggerISR(int ch) {
	LancChannel &C = lanc[ch];

	uint32_t now = micros();
	uint32_t gap = now - C.last_falling_edge;
	C.last_falling_edge = now;

	if (!C.active) return;

	switch (C.state) {

		case SEARCHING_SYNC:
			if (gap >= SYNC_GAP_MIN_US && C.cmdPending) {
					C.state   = PROCESSING_BYTE;
          // Record start time for bit-bang engine
          lancStartTime = micros();
          lancBitBangActive = true;
					// Inject start bit immediately
					digitalWrite(C.txPin, LOW);
			}
			break;

		case WAITING_START:
				// For future multi-byte sequences
				break;

		default:
				break;
	}
}

// Attach ISRs
void lancTriggerISR0() { lancTriggerISR(0); }
void lancTriggerISR1() { lancTriggerISR(1); }
void lancTriggerISR2() { lancTriggerISR(2); }

// ─────────────────────────────────────────────────────────────
// Timer ISR — pure bit clock + status scaffolding
// ─────────────────────────────────────────────────────────────
void lancTimerISR(timer_callback_args_t *) {

	if (activeCam < 0) {
			lancTimer.stop();
			return;
	}

	LancChannel &C = lanc[activeCam];


	// Status read scaffolding (bytes 2–7) — not active yet, but ready
	if (C.state == READING_STATUS) {
		// Example skeleton: sample at mid-bit and fill rxBuf[2..7]
		static uint8_t tick = 0;
		tick++;

		if ((tick & 1) == 1) {
			uint8_t bitIdx = C.statusBitIdx;  // 0=start, 1..8=data, 9=stop

			if (bitIdx >= 1 && bitIdx <= 8) {
					uint8_t bit = digitalRead(C.rxPin) ? 1 : 0;
					C.rxBuf[C.statusByteIdx] |= (bit << (bitIdx - 1));
			}

			C.statusBitIdx++;

			if (C.statusBitIdx >= 10) {
				// Finished one status byte
				C.statusBitIdx = 0;
				C.statusByteIdx++;

				if (C.statusByteIdx > 7) {
						// Done bytes 2..7
						C.state = SEARCHING_SYNC;
						lancTimer.stop();
						tick = 0;
				}
			}
		}
	}
}

// ─── Helpers ─────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
// Bit-bang LANC injection engine (bytes 0–1)
// ─────────────────────────────────────────────────────────────
void processLancBitBang() {

    if (!lancBitBangActive || activeCam < 0) return;

    LancChannel &C = lanc[activeCam];

    // We will inject exactly 2 bytes = 20 bits
    const uint8_t totalBits = 20;

    uint32_t t0 = lancStartTime;

    // Bit 0 (start bit) is already LOW from the ISR

    for (uint8_t bit = 1; bit < totalBits; bit++) {

        // Wait for next bit boundary
        uint32_t target = t0 + (bit * BIT_TIME_US);
        while (micros() < target) {
            // interrupts remain enabled, serial handshake still works
        }

        // Determine which byte and which bit we are sending
        uint8_t byteIdx = bit / 10;          // 0 or 1
        uint8_t bitIdx  = bit % 10;          // 0=start, 1..8=data, 9=stop

        if (bitIdx == 0) {
            digitalWrite(C.txPin, LOW);   // start bit
        }
        else if (bitIdx >= 1 && bitIdx <= 8) {
            uint8_t b = C.txBuf[byteIdx];
            uint8_t dataBit = (b >> (bitIdx - 1)) & 1;
            digitalWrite(C.txPin, dataBit ? HIGH : LOW);
        }
        else {
            digitalWrite(C.txPin, HIGH);  // stop bit
        }
    }

    // Injection complete
    lancBitBangActive = false;
    C.cmdPending = false;
    C.state = SEARCHING_SYNC;
    lancPacketComplete = true;

    // Multi-frame repeat support
    if (lancFrameRepeatRemaining > 0) {
        lancFrameRepeatRemaining--;

        if (lancFrameRepeatRemaining > 0) {
            C.cmdPending = true;
            C.state = SEARCHING_SYNC;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// FRAME PARSING ROUTINE
// Call from loop(); accumulates chars, acts on complete '<...>' frames.
//
// Supported frame formats:
//   <CamId D>            select camera + pan/tilt command:  e.g. <1U> <2L> <3S>
//   <CamId Z B1 B2>      raw LANC, two hex bytes:   e.g. <1Z 28 00>
//   (spaces in LANC frame are optional / ignored)
// ═══════════════════════════════════════════════════════════════════════════════
void parseSerialInput() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    switch (currentState) {
    case FrameState::IDLE:
      if (c == STX) {
        rxDataLen   = 0;
        memset(rxDataBuf, 0, sizeof(rxDataBuf));
        Serial.println(rxAckSTX);   // ack start marker
        currentState = FrameState::CID;
      }
      // anything before STX is silently ignored
      break;

    case FrameState::CID:
      validCamId = isValidCamId(c);
      if (validCamId) {
        rxDataBuf[rxDataLen++] = c;
        Serial.println(rxAckCID);   // ack CID
        currentState = FrameState::CMD;
      }
      else {
        currentState = FrameState::IDLE;
      } 
      break;

    case FrameState::CMD:
      validCmd = isValidCmd(c);
      if (validCmd) {
        rxDataBuf[rxDataLen++] = c;
        Serial.println(rxAckCMD);   // ack CID
        currentState = FrameState::DATA;
      }
	  else {
        currentState = FrameState::IDLE;
	  }
    break;

    case FrameState::DATA:
      if (c == ETX) {
        Serial.println(rxAckETX);   // ack end marker
        processFrame(rxDataBuf,rxDataLen);
        rxDataLen = 0;
        currentState = FrameState::IDLE;
      }
      else { 
        // capture outgoing LANC data   
        // Buffer overflow guard
        if (rxDataLen < RX_DATA_BUF_SIZE - 1) {
          rxDataBuf[rxDataLen++] = c;
          rxDataBuf[rxDataLen]   = '\0';
          Serial.println(rxAckCh);    // ack each character received
        }
      }      
      break;
    }
  }
}

// ─── Frame processor ─────────────────────────────────────────────────────────
// Called once a complete frame (contents between < >) has been collected.
void processFrame(const char* buf, uint8_t len) {
  if (len < 2) return;   // minimum: CamId + command

  // --- Byte 0: camera ID ---
  char camChar = buf[0];
  volatile int8_t camIdx = 0;
  
  if      (camChar == cam1Id) camIdx = 0;
  else if (camChar == cam2Id) camIdx = 1;
  else if (camChar == cam3Id) camIdx = 2;

  if ((camIdx >= NUM_CAMERAS) 
  ||
  (camIdx < 0)) {
    Serial.println("ERR:CAMID");
    return;
  }

  // Switch active camera if changed; release pan/tilt of previous
	selectLancChannel(camIdx);

  // --- Byte 1: command character ---
  char cmd = buf[1];

  switch (cmd) {

    case CMD_PAN_LEFT:
      actuatePanTilt((uint8_t)camIdx, panLeftPin[camIdx]);
      break;

    case CMD_PAN_RIGHT:
      actuatePanTilt((uint8_t)camIdx, panRightPin[camIdx]);
      break;

    case CMD_TILT_UP:
      actuatePanTilt((uint8_t)camIdx, tiltUpPin[camIdx]);
      break;

    case CMD_TILT_DOWN:
      actuatePanTilt((uint8_t)camIdx, tiltDownPin[camIdx]);
      break;

    case CMD_PAN_STOP:
      releasePanTilt((uint8_t)camIdx);
      break;

    case CMD_LANC:
      // Expect 2 more hex bytes in buf[2..3] and buf[4..5] (spaces optional)
      // Strip spaces to collect hex chars
      {
        char hexStr[5] = { 0 };
        uint8_t hi = 2;
        uint8_t hIdx = 0;
        while (hi < len && hIdx < 4) {
          if (buf[hi] != ' ') hexStr[hIdx++] = buf[hi];
          hi++;
        }
        if (hIdx < 4) { 
					Serial.println("ERR:LANC_DATA");
					break; 
				}

        char temp1[3] = { hexStr[0], hexStr[1], '\0' };
        uint8_t b1 = (uint8_t)strtol(temp1, NULL, 16);
        char temp2[3] = {hexStr[2], hexStr[3], '\0'};
        uint8_t b2 = (uint8_t)strtol(temp2, NULL, 16);

        lancCmdReceived = true;
        queueLancCommand(b1, b2, 3);
      }
      break;

    default:
      Serial.println("ERR:CMD");
      break;
  }
}

// ─── LANC Timer Init ──────────────────────────────────────────────────
void initLancTimer() {
  float freqHz = 1.0e6f / (float)HALF_BIT_TIME_US;  // ~19231 Hz

  uint8_t timerType = GPT_TIMER;
  int8_t  channel   = lancTimer.get_available_timer(timerType);
  if (channel < 0) {
    Serial.println("ERR: No AGT timer available");
    return;
  }

  if (!lancTimer.begin(TIMER_MODE_PERIODIC,
                       timerType,
                       channel,
                       freqHz,
                       0.0f,
                       lancTimerISR)) {
    Serial.println("ERR: lancTimer.begin() failed");
    return;
  }

  lancTimer.setup_overflow_irq();
  lancTimer.open();
  lancTimer.stop();
}

// ═══════════════════════════════════════════════════════════════════════════════
// setup()
// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
  initHardware();   // defined in init routine above
}

// ═══════════════════════════════════════════════════════════════════════════════
// loop()
// ═══════════════════════════════════════════════════════════════════════════════
void loop() {

	// 1. Run bit-bang engine if active
	processLancBitBang();

	// 2. Service incoming USB-serial frames from host
	parseSerialInput();

	// 3. Handle completed LANC packet + multi-frame repeat
	if (lancPacketComplete) {
		Serial.println(rxAckCmplt);
		lancPacketComplete = false;

		if (lancFrameRepeatRemaining > 0) {
			lancFrameRepeatRemaining--;

			if (lancFrameRepeatRemaining > 0 && activeCam >= 0) {
					// Re-arm same command for next frame
					lanc[activeCam].cmdPending = true;
					lanc[activeCam].state      = SEARCHING_SYNC;
			}
		}
	}	
}

// ═══════════════════════════════════════════════════════════════════════════════
// INIT ROUTINE
// ═══════════════════════════════════════════════════════════════════════════════
void initHardware() {
  // --- Serial (USB virtual COM) ---
  Serial.begin(115200);
  delay(2500); 
  // Wait for host to open port (native USB)
  uint32_t t = millis();
  while (!Serial && (millis() - t < 5000));

  // --- Pan/tilt pins: all HIGH-Z by default ---
  releaseAllPanTilt();

	lanc[0] = {CAM1_LANC_SIG_IN, CAM1_LANC_CMD_OUT, micros(), false, false, {0}, {0}, 0, 0, SEARCHING_SYNC};
	lanc[1] = {CAM2_LANC_SIG_IN, CAM2_LANC_CMD_OUT, micros(), false, false, {0}, {0}, 0, 0, SEARCHING_SYNC};
	lanc[2] = {CAM3_LANC_SIG_IN, CAM3_LANC_CMD_OUT, micros(), false, false, {0}, {0}, 0, 0, SEARCHING_SYNC};
	
	for (int i = 0; i < 3; i++) {
			pinMode(lanc[i].rxPin, INPUT_PULLUP);
			pinMode(lanc[i].txPin, OUTPUT);
			digitalWrite(lanc[i].txPin, LOW);
	}

	attachInterrupt(digitalPinToInterrupt(lanc[0].rxPin), lancTriggerISR0, FALLING);
	attachInterrupt(digitalPinToInterrupt(lanc[1].rxPin), lancTriggerISR1, FALLING);
	attachInterrupt(digitalPinToInterrupt(lanc[2].rxPin), lancTriggerISR2, FALLING);

	initLancTimer();

  // --- Application state ---
  activeCam     = -1;
  rxDataLen     = 0;
  lancPacketComplete = false;
  lancFrameRepeatCount     = 1;
  lancFrameRepeatRemaining = 0;
	
  // --- Host handshake ---
  // Wait for HostListeningCode '!'
  while (true) {
    if (Serial.available()) {
      char c = (char)Serial.read();
      if (c == HostListeningCode) {
        Serial.println("Arduino LANC to USB-serial interface v1.0");
        break;
      }
    }
  }
}

void flushSerialBuffer() {
  // flush the serial input buffer
  while (Serial.available() > 0) {
    Serial.read();
  }
}
