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

// ─── Host Communication Protocol constants ──────────────────────────────────────────────────────
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
#define CMD_LANC_STOP   'Y'  

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

// LANC constants


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
const uint32_t BIT_TIME_US = 104; // LANC bit time (~104 µs)
const uint32_t HALF_BIT_TIME_US = 52;
const uint32_t FRAME_SYNC_MIN_US = 5000;  // gap before byte 0 (~1 ms)

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
#define NUM_COMMANDS 7
const uint8_t validCmdValues[NUM_COMMANDS] = { CMD_PAN_LEFT, CMD_PAN_RIGHT, CMD_TILT_UP, CMD_TILT_DOWN, CMD_PAN_STOP, CMD_LANC, CMD_LANC_STOP }; 
volatile uint8_t lancActiveSigPin = CAM1_LANC_SIG_IN;

// ─────────────────────────────────────────────────────────────
// State Machine States
// ─────────────────────────────────────────────────────────────

// ─── Lanc State (interrupt-driven, one camera active) ──────────────────
enum LancState {
    SEARCHING_SYNC,
    BYTE0_START,
    BYTE0_DATA,
    BYTE0_STOP,
    BYTE1_START,
    BYTE1_DATA,
    BYTE1_STOP
};	
// ─── Host message frame state (tight handshake -- 1-2 characters, 1 Ack) ──────────────────

enum FrameState {
    IDLE,
    CID,
    CMD,
    DATA
};

// ─── Zoom State ──────────────────

enum ZoomState {
    ZOOM_IDLE,
    ZOOM_ACTIVE,
		ZOOM_STOP
};

volatile ZoomState zoomState = ZOOM_IDLE;
volatile uint8_t zoomSpeed = 0x08;   // default medium speed
	
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
volatile bool trap = false;

// ─── Bit-Bang control ───────────────────────────────────────────────────────────
volatile LancState lancState        = SEARCHING_SYNC;
volatile uint8_t currentByte   = 0;      // 0..7 
volatile bool lancBitBangActive = false;
volatile uint32_t lancStartTime = 0;     // start bit time for current byte
// Optional: status buffer for bytes 2–7
uint8_t lancStatus[8];

// Pending command loaded by main loop

uint8_t b1 = 0;
uint8_t b2 = 0;

volatile bool    lancCmdReceived = false;   
volatile bool    lancCmdPending = false;


// variables tracking the LANC start timing state
volatile bool actionComplete = false;

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
void queueLancCommand(uint8_t b0, uint8_t b1) {
    if (activeCam < 0) return;

    lanc[activeCam].txBuf[0] = b0;
    lanc[activeCam].txBuf[1] = b1;
    lanc[activeCam].cmdPending = true;
    lancPacketComplete = false;
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

// ─────────────────────────────────────────────────────────────
// Public API: set continuous zoom state
// ─────────────────────────────────────────────────────────────
void startZoom()  { zoomState = ZOOM_ACTIVE;  }
void stopZoom()     { zoomState = ZOOM_IDLE; }


// ─── ISRs ─────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
// LANC edge ISR: frame sync + byte start sync
// Called on every falling edge of the LANC bus
// ─────────────────────────────────────────────────────────────
void lancTriggerISR(int ch) {
	uint32_t now = micros();
	static uint32_t lastEdge = 0;
	uint32_t gap = now - lastEdge;
	lastEdge = now;

	if (activeCam < 0) return;
	LancChannel &C = lanc[ch];

	switch (lancState) {

	case SEARCHING_SYNC:
		// FRAME SYNC: long gap → start of byte 0
		if (gap >= FRAME_SYNC_MIN_US) {
				currentByte   = 0;
				lancStartTime = now;
				lancState     = BYTE0_START;
				
		}
		break;
		
  // BYTE SYNC: start of byte 1 (short gap after byte 0 stop/padding)
	case BYTE0_STOP:
		// After stop bit + padding, next falling edge is start of next byte
		// No gap check needed; we already know we're inside a frame
			currentByte = 1;
			lancStartTime = now;
			lancState     = BYTE1_START;
		break;

	default:
			break;
			
	    // We do not sync bytes 2–7 here; status reading is optional and separate		
	}		
}

// Attach ISRs
void lancTriggerISR0() { lancTriggerISR(0); }
void lancTriggerISR1() { lancTriggerISR(1); }
void lancTriggerISR2() { lancTriggerISR(2); }

// ─── Helpers ─────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────
// inject one byte (8 data bits, bits 1..8)
// ─────────────────────────────────────────────────────────────
void injectByte(uint8_t byteIndex) {
    if (activeCam < 0) return;
    LancChannel &C = lanc[activeCam];

    uint32_t t0 = lancStartTime;

    for (uint8_t bitIdx = 1; bitIdx <= 8; bitIdx++) {

        uint32_t target = t0 + (bitIdx * BIT_TIME_US);
        while (micros() < target) {
            // interrupts remain enabled
        }

        uint8_t b       = C.txBuf[byteIndex];
        uint8_t dataBit = (b >> (bitIdx - 1)) & 1;

        // Your hardware: HIGH = bus LOW, LOW = bus HIGH
        if (dataBit == 0) {
            digitalWrite(C.txPin, LOW);   // bus HIGH
        } else {
            digitalWrite(C.txPin, HIGH);    // bus LOW
        }
    }

    // Release bus before stop bit
    digitalWrite(C.txPin, LOW);
}

// ─────────────────────────────────────────────────────────────
// Optional: read status bytes 2–7 (called once per frame)
// This does NOT affect injection timing.
// You can call it after lancPacketComplete == true.
// ─────────────────────────────────────────────────────────────
void readStatusBytes() {
    if (activeCam < 0) return;
    LancChannel &C = lanc[activeCam];

    // This is a simple placeholder; you can refine timing with a separate
    // bit-level reader if you want exact status decoding later.
    // For now, we just sample bytes 2–7 at a coarse level or skip entirely.
    // (Real status reading would mirror injectByte() but with digitalRead.)
}

// ─────────────────────────────────────────────────────────────
//  Bit-bang LANC injection engine (bytes 0–1, data bits only)
// ─────────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────
// Bit-bang engine: inject bytes 0–1 per frame
// Call this frequently from loop()
// ─────────────────────────────────────────────────────────────
void processLancBitBang() {

    if (activeCam < 0) return;
    LancChannel &C = lanc[activeCam];

    switch (lancState) {

    case BYTE0_START:
        lancBitBangActive = true;
        lancState         = BYTE0_DATA;
        break;

    case BYTE0_DATA:
        if (!lancBitBangActive) return;
        injectByte(0);
        lancBitBangActive = false;
        lancState         = BYTE0_STOP;
        break;

    case BYTE0_STOP:
        // Wait for ISR to detect start of byte 1
        break;

    case BYTE1_START:
        lancBitBangActive = true;
        lancState         = BYTE1_DATA;
        break;

    case BYTE1_DATA:
        if (!lancBitBangActive) return;
        injectByte(1);
        lancBitBangActive = false;
        lancState         = BYTE1_STOP;
        break;

    case BYTE1_STOP:
        // Command complete for this frame
        C.cmdPending        = false;
        lancPacketComplete  = true;

        // Return to SEARCHING_SYNC immediately so we see the next frame gap
        lancState     = SEARCHING_SYNC;
        currentByte   = 0;
        break;

    case SEARCHING_SYNC:
    default:
        break;
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
		  // For safety, terminate Zoom in case it's still active
			zoomState = ZOOM_IDLE;
  		// Acknowledge received command
			Serial.println(rxAckCmplt);
      break;

    case CMD_PAN_RIGHT:
      actuatePanTilt((uint8_t)camIdx, panRightPin[camIdx]);
		  // For safety, terminate Zoom in case it's still active
			zoomState = ZOOM_IDLE;
  		// Acknowledge received command
			Serial.println(rxAckCmplt);
      break;

    case CMD_TILT_UP:
      actuatePanTilt((uint8_t)camIdx, tiltUpPin[camIdx]);
		  // For safety, terminate Zoom in case it's still active
			zoomState = ZOOM_IDLE;
  		// Acknowledge received command
			Serial.println(rxAckCmplt);
      break;

    case CMD_TILT_DOWN:
      actuatePanTilt((uint8_t)camIdx, tiltDownPin[camIdx]);
		  // For safety, terminate Zoom in case it's still active
			zoomState = ZOOM_IDLE;
  		// Acknowledge received command
			Serial.println(rxAckCmplt);
      break;

    case CMD_PAN_STOP:
      releasePanTilt((uint8_t)camIdx);
		  // For safety, terminate Zoom in case it's still active
			zoomState = ZOOM_IDLE;
  		// Acknowledge received command
			Serial.println(rxAckCmplt);
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
        b1 = (uint8_t)strtol(temp1, NULL, 16);
        char temp2[3] = {hexStr[2], hexStr[3], '\0'};
        b2 = (uint8_t)strtol(temp2, NULL, 16);

        lancCmdReceived = true;
				zoomState = ZOOM_ACTIVE;
				// Acknowledge received command
				Serial.println(rxAckCmplt);
      }
      break;
			
    case CMD_LANC_STOP:
		  // Intentional zoom termination:
		  zoomState = ZOOM_STOP;
  		// Acknowledge received command
			Serial.println(rxAckCmplt);
      break;

    default:
      Serial.println("ERR:CMD");
      break;
  }
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

	// Maintain continuous zoom
	if (zoomState == ZOOM_ACTIVE && !lancBitBangActive && lancCmdReceived) {
    // If we are not currently injecting and no new host command arrived,
    // send another zoom command for the next frame.
		queueLancCommand(b1, b2);	
	}

	// 1. Run bit-bang engine if active
	processLancBitBang();

	// 2. Service incoming USB-serial frames from host
	parseSerialInput();

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

  // --- Application state ---
  activeCam     = -1;
  rxDataLen     = 0;
  lancPacketComplete = false;
	
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
