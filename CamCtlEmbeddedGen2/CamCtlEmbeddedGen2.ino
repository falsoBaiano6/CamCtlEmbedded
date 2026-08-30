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
#include <WiFiS3.h>
#include <WiFiUdp.h>

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
const uint32_t BIT_TIME = 104;
const uint32_t HALF_BIT_TIME = 52;
const uint32_t SYNC_GAP_MIN = 5000; // 5ms sync preamble threshold

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
volatile uint8_t activeLancSigPin = CAM1_LANC_SIG_IN;
volatile enum { SEARCHING_SYNC, WAITING_START, PROCESSING_BYTE } lanc_state = SEARCHING_SYNC;
volatile uint32_t last_falling_edge = 0;
volatile uint8_t byte_counter = 0;
volatile uint8_t bit_counter = 0;

// ─── LANC timing ──────────────────────────────────────────────────────────────
// LANC runs at 9600 baud → 1 bit = 104.167 µs.
// Timer fires every half-bit (52 µs) so we can sample at mid-bit as well.
// Each LANC frame = start bit + 8 data bits + stop bit = 10 half-bit pairs = 20 ticks.
// Two bytes per frame → 40 ticks total.
#define LANC_HALF_BIT_US  52u
#define LANC_BITS_PER_BYTE 10u   // start + 8 data + stop
#define LANC_TICKS_PER_BYTE (LANC_BITS_PER_BYTE * 2u)   // 20 half-bit ticks

// ─── Application state ────────────────────────────────────────────────────────
int8_t  activeCam      = -1;   // 0-based; -1 = none selected
bool    hostConnected  = false;

// ─── LANC ISR state ───────────────────────────────────────────────────────────
volatile bool    lancBusy       = false;
volatile uint8_t lancTxBuf[2]   = { 0x00, 0x00 };  // up to 2 bytes per frame
volatile uint8_t lancTxLen      = 0;                // 1 or 2 bytes
volatile uint8_t lancByteIdx    = 0;                // which byte we are on
volatile uint8_t lancBitTick    = 0;                // half-bit tick within current byte
volatile uint8_t lancActiveCmdPin = 0xFF;           // pin currently being driven

// ─── LANC frame state (interrupt-driven, one camera active) ──────────────────

enum FrameState {
    IDLE,
    CID,
    CMD,
    DATA
};

FrameState currentState = IDLE;
static bool validCamId = false;
static bool validCmd = false;

volatile uint8_t  lancCmdByte    = 0x00;   // command byte to transmit
volatile uint8_t  lancCmdByte2   = 0x00;   // second LANC byte (if needed)

// Pending command loaded by main loop

volatile bool    lancCmdReceived = false;   
volatile bool    lancCmdPending = false;
volatile uint8_t lancPendingBuf[2];
volatile uint8_t lancPendingLen = 0;
volatile uint8_t lancCmdPinState = 0;

// variables tracking the LANC start timing state
volatile unsigned long lancTime = 0;
volatile bool lancPinHigh = false;
volatile bool lancPacketComplete = true;

FspTimer lancTimer;

// ─── Serial receive buffer ────────────────────────────────────────────────────
#define RX_DATA_BUF_SIZE 8
#define RX_LANC_DATA_START_IX 2
static char  rxDataBuf[RX_DATA_BUF_SIZE];
static uint8_t rxDataBufIdx  = 0;
static bool  inFrame     = false;
static uint8_t rxDataLen = 0;

// LANC char buffer
#define NUM_LANC_DATA_CHARS 5
#define NUM_LANC_BYTES_TO_WRITE 2
static char lancDataCharBuf[NUM_LANC_DATA_CHARS];
static boolean lancCmd[16];

// ─── ISRs ─────────────────────────────────────────────────────────────────

// ═══════════════════════════════════════════════════════════════════════════════
// LANC timer ISR  (fires every LANC_HALF_BIT_US)
// ═══════════════════════════════════════════════════════════════════════════════
void lancTimerISR(timer_callback_args_t __attribute__((unused)) *args) {
  if (!lancBusy) {
    // Check if main loop issued a new command
    if (lancCmdPending) {
      lancTxBuf[0]    = lancPendingBuf[0];
      lancTxBuf[1]    = lancPendingBuf[1];
      lancTxLen       = lancPendingLen;
      lancByteIdx     = 0;
      lancBitTick     = 0;
      lancCmdPending  = false;
      lancBusy        = true;
      // pin is already set OUTPUT + LOW (idle) in queueLancCommand()
    }
    return;
  }

  uint8_t pin  = lancActiveCmdPin;
  uint8_t tick = lancBitTick;           //
  uint8_t b    = lancTxBuf[lancByteIdx];

  // We act only on even ticks (the "set" tick); odd ticks are the hold phase.
  if ((tick & 0x01) == 0) {
    uint8_t bitIdx = tick >> 1;     // 0=start, 1-8=data, 9=stop

    if (bitIdx == 0) {
      // Start bit — let camera master drive the bus

    } else if (bitIdx <= 8) {
      // Data bits, LSB first
      uint8_t dataBit = (b >> (bitIdx - 1)) & 0x01;
      digitalWrite(pin, dataBit ? HIGH : LOW);

    } else {
      // Stop bit — let camera master drive the bus      
    }
  }

  lancBitTick++;

  // Finished all ticks for this byte?
  if (lancBitTick >= LANC_TICKS_PER_BYTE) {
    lancBitTick = 0;
    lancByteIdx++;

    if (lancByteIdx >= NUM_LANC_BYTES_TO_WRITE /*lancTxLen */) {
      // We only write the first 2 bytes — return to idle
			// Frame complete, reset state engine to find the next synchronization preamble
			lanc_state = SEARCHING_SYNC;
			last_falling_edge = micros();
			lancTimer.stop();
			lancPacketComplete = true;
			Serial.println(rxAckCmplt);
      lancBusy = false;
    }
		else {
			// Prepare to capture the start bit of the next sequential byte
			lanc_state = WAITING_START;
		}
		attachInterrupt(digitalPinToInterrupt(activeLancSigPin), lancTriggerIsr, FALLING);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// LANC trigger ISR  (External Falling Edge Trigger)
// ═══════════════════════════════════════════════════════════════════════════════
void lancTriggerIsr() {
	uint32_t now = micros();

	if (lanc_state == SEARCHING_SYNC) {
		// Check if the idle period since the last edge matches the >5ms LANC sync gap
		if ((now - last_falling_edge) >= SYNC_GAP_MIN) {
			lanc_state = PROCESSING_BYTE;
			byte_counter = 0;
			start_bit_timer(BIT_TIME + HALF_BIT_TIME); // Sample middle of Bit 0 (156us)
		}
		last_falling_edge = now;
	}
	else if (lanc_state == WAITING_START) {
		// Detected the start bit falling edge of bytes 1 through 7
		detachInterrupt(digitalPinToInterrupt(activeLancSigPin)); // Block further edge triggers
		lanc_state = PROCESSING_BYTE;
		start_bit_timer(BIT_TIME + HALF_BIT_TIME); // Sync timer to middle of Bit 0
	}
}

// ─── Helpers ─────────────────────────────────────────────────────────────────

// Helper to configure and kick off the hardware timer interval
void start_bit_timer(uint32_t microsecond_delay) {
	bit_counter = 0;
	lancTimer.set_period((double)microsecond_delay);
	lancTimer.open();
	lancTimer.start();
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

// Function to safely swap pins at runtime
void changeLancPin(byte newPin) {
  // 1. Remove interrupt from the old pin
  detachInterrupt(digitalPinToInterrupt(activeLancSigPin));
  
  // 2. Set the new pin variable
  activeLancSigPin = newPin;
  
  // 3. Initialize and attach the new pin
  initLancPin(activeLancSigPin);
}

// Helper to configure a pin and set baseline states
void initLancPin(byte pin) {
  pinMode(pin, INPUT);
  
  // Clear states before attaching to avoid false triggers
  noInterrupts();
  if (digitalRead(pin) == HIGH) {
    lancPinHigh = true;
    lancTime = micros();
  } else {
    lancPinHigh = false;
  }
  interrupts();

  // Attach the shared ISR function to the new pin
  //attachInterrupt(digitalPinToInterrupt(pin), lancTriggerIsr, FALLING);
}

// Non-blocking evaluation function
bool checkLancReady() {
  noInterrupts();
  bool isHigh = lancPinHigh;
  unsigned long startTime = lancTime;
  interrupts();

  if (!isHigh) {
    return false; 
  }
  
  if (micros() - startTime >= 5000) {
    return true; 
  }
  
  return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
// LANC TIMER STOP / RESTART
// ═══════════════════════════════════════════════════════════════════════════════

// Stop the LANC timer and leave the CMD pin in idle-HIGH state.
// Safe to call from loop(); do NOT call from within lancTimerISR.
void stopLancTimer() {
  lancTimer.stop();
  lancTimer.close();

  // Reset ISR state so a restart begins cleanly
  lancBusy       = false;
  lancCmdPending = false;
  lancBitTick    = 0;
  lancByteIdx    = 0;

  // Drive all CMD pins LOW
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(lancCmdPin[i], LOW);
  }
}

// Restart the LANC timer after stopLancTimer().
// Reuses the same FspTimer instance and ISR.
void restartLancTimer() {
  float freqHz = 1.0e6f / (float)LANC_HALF_BIT_US;   // ~19231 Hz
  if (lancTimer.begin(TIMER_MODE_PERIODIC,
                      AGT_TIMER,
                      0,          // AGT channel 0
                      freqHz,
                      0.0f,
                      lancTimerISR)) {
    lancTimer.setup_overflow_irq();
    lancTimer.open();
    lancTimer.start();
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
  int8_t camIdx = -1;
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
  if (camIdx != activeCam) {
    if (activeCam >= 0) releasePanTilt((uint8_t)activeCam);
    activeCam = camIdx;
    changeLancPin(activeCam);
  }

  // --- Byte 1: command character ---
  char cmd = buf[1];

  switch (cmd) {

    case CMD_PAN_LEFT:
      actuatePanTilt((uint8_t)camIdx, panLeftPin[camIdx]);
			attachInterrupt(digitalPinToInterrupt(activeCam), lancTriggerIsr, FALLING);
      break;

    case CMD_PAN_RIGHT:
      actuatePanTilt((uint8_t)camIdx, panRightPin[camIdx]);
			attachInterrupt(digitalPinToInterrupt(activeCam), lancTriggerIsr, FALLING);
      break;

    case CMD_TILT_UP:
      actuatePanTilt((uint8_t)camIdx, tiltUpPin[camIdx]);
			attachInterrupt(digitalPinToInterrupt(activeCam), lancTriggerIsr, FALLING);
      break;

    case CMD_TILT_DOWN:
      actuatePanTilt((uint8_t)camIdx, tiltDownPin[camIdx]);
			attachInterrupt(digitalPinToInterrupt(activeCam), lancTriggerIsr, FALLING);
      break;

    case CMD_PAN_STOP:
      releasePanTilt((uint8_t)camIdx);
			attachInterrupt(digitalPinToInterrupt(activeCam), lancTriggerIsr, FALLING);
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
        queueLancCommand(b1, b2, 2);
				attachInterrupt(digitalPinToInterrupt(activeCam), lancTriggerIsr, FALLING);
      }
      break;

    default:
      Serial.println("ERR:CMD");
      break;
  }
}

// ─── Queue a LANC command for the ISR ────────────────────────────────────────
void queueLancCommand(uint8_t b1, uint8_t b2, uint8_t len) {
  if (activeCam < 0) return;
  // Spin-wait if ISR is still transmitting (very short — max 2×10 bits at 9600)
  while (lancBusy);
  lancCmdPinState      = lancCmdPin[activeCam];
  lancPendingBuf[0]   = b1;
  lancPendingBuf[1]   = b2;
  lancPendingLen      = len;
  lancCmdPending      = true;   // ISR picks this up on next tick
}

// ─── LANC Timer Init (fixed) ──────────────────────────────────────────────────
void initLancTimer() {
  float freqHz = 1.0e6f / (float)LANC_HALF_BIT_US;  // ~19231 Hz

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
  lancTimer.start();
  //Serial.print("LANC timer OK on AGT ch");
  //Serial.println(channel);
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
  // 1. Service incoming USB-serial frames from host
  parseSerialInput();

  // // 2. If host disconnects (DTR dropped), reset state and wait for reconnect
  // if (hostConnected && !Serial) {
  //   hostConnected = false;
  //   activeCam     = -1;
  //   releaseAllPanTilt();
  //   Serial.println("DISCONNECTED");
  //   // Spin until host reconnects and sends '!'
  //   while (true) {
  //     if (Serial && Serial.available()) {
  //       char c = (char)Serial.read();
  //       if (c == HostListeningCode) {
  //         hostConnected = true;
  //         Serial.println("Arduino LANC to USB-serial interface v1.0");
  //         break;
  //       }
  //     }
  //   }
  // }

  // 3. No other blocking work here — LANC TX is handled entirely by lancTimerISR,
  //    and pan/tilt pins are set directly inside processFrame().
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

  // --- LANC CMD pins: OUTPUT, idle LOW ---
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(lancCmdPin[i], OUTPUT);
    digitalWrite(lancCmdPin[i], LOW);
  }

  // --- LANC SIG_IN pins: INPUT, pull-up (LANC bus is active-low) ---
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(lancSigPin[i], INPUT_PULLUP);
  }

  initLancTimer();

  // --- Application state ---
  activeCam     = -1;
  hostConnected = false;
  inFrame       = false;
  rxDataLen     = 0;

  initLancPin(activeLancSigPin);

  // --- Host handshake ---
  // Wait for HostListeningCode '!'
  while (true) {
    if (Serial.available()) {
      char c = (char)Serial.read();
      if (c == HostListeningCode) {
        Serial.println("Arduino LANC to USB-serial interface v1.0");
        hostConnected = true;
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
