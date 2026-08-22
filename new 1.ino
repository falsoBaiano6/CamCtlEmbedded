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

#include "FspTimer.h"   // RA4M1 FSP timer wrapper bundled with UNO R4 core

// ─── Protocol constants ──────────────────────────────────────────────────────
#define HostListeningCode '!'
#define STX               '<'
#define ETX               '>'

#define rxAckSTX  "FE"
#define rxAckCh   "AA"
#define rxAckETX  "EF"
#define rxAckCID  '@'

#define cam1Id    '1'
#define cam2Id    '2'
#define cam3Id    '3'

// ─── Pan/Tilt command codes (placeholders — assign when selected) ─────────────
#define CMD_PAN_LEFT    'L'
#define CMD_PAN_RIGHT   'R'
#define CMD_TILT_UP     'U'
#define CMD_TILT_DOWN   'D'
#define CMD_PAN_STOP    'S'   // release all pan/tilt for active camera

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
// Slowest: 28 10
// Medium: 28 14
// Fastest: 28 1E

// ─── Pin lookup tables (index: 0=CAM1, 1=CAM2, 2=CAM3) ───────────────────────
const uint8_t lancCmdPin[3]  = { CAM1_LANC_CMD_OUT, CAM2_LANC_CMD_OUT, CAM3_LANC_CMD_OUT };
const uint8_t lancSigPin[3]  = { CAM1_LANC_SIG_IN,  CAM2_LANC_SIG_IN,  CAM3_LANC_SIG_IN  };
const uint8_t panLeftPin[3]  = { CAM1_PAN_LEFT,  CAM2_PAN_LEFT,  CAM3_PAN_LEFT  };
const uint8_t panRightPin[3] = { CAM1_PAN_RIGHT, CAM2_PAN_RIGHT, CAM3_PAN_RIGHT };
const uint8_t tiltUpPin[3]   = { CAM1_TILT_UP,   CAM2_TILT_UP,   CAM3_TILT_UP   };
const uint8_t tiltDownPin[3] = { CAM1_TILT_DOWN, CAM2_TILT_DOWN, CAM3_TILT_DOWN };

const uint8_t allPanTiltPins[] = {
  CAM1_PAN_LEFT, CAM1_PAN_RIGHT, CAM1_TILT_UP, CAM1_TILT_DOWN,
  CAM2_PAN_LEFT, CAM2_PAN_RIGHT, CAM2_TILT_UP, CAM2_TILT_DOWN,
  CAM3_PAN_LEFT, CAM3_PAN_RIGHT, CAM3_TILT_UP, CAM3_TILT_DOWN
};

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
volatile uint8_t  lancCmdByte    = 0x00;   // command byte to transmit
volatile uint8_t  lancCmdByte2   = 0x00;   // second LANC byte (if needed)
// Pending command loaded by main loop
volatile bool    lancCmdPending = false;
volatile uint8_t lancPendingBuf[2];
volatile uint8_t lancPendingLen = 0;

FspTimer lancTimer;

// ─── Serial receive buffer ────────────────────────────────────────────────────
#define RX_BUF_SIZE 8
static char  rxBuf[RX_BUF_SIZE];
static uint8_t rxBufIdx  = 0;
static bool  inFrame     = false;


// ═══════════════════════════════════════════════════════════════════════════════
// setup()
// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
  initHardware();   // defined in init routine above
}

// ─── Helpers ─────────────────────────────────────────────────────────────────

// ═══════════════════════════════════════════════════════════════════════════════
// LANC timer ISR  (fires every LANC_HALF_BIT_US)
// ═══════════════════════════════════════════════════════════════════════════════
void lancTimerISR(timer_callback_args_t __attribute__((unused)) *args) {
  if (!lancBusy) {
    // Check if main loop queued a new command
    if (lancCmdPending) {
      lancTxBuf[0]    = lancPendingBuf[0];
      lancTxBuf[1]    = lancPendingBuf[1];
      lancTxLen       = lancPendingLen;
      lancByteIdx     = 0;
      lancBitTick     = 0;
      lancCmdPending  = false;
      lancBusy        = true;
      // pin is already set OUTPUT + HIGH (idle) in queueLancCommand()
    }
    return;
  }

  uint8_t pin  = lancActiveCmdPin;
  uint8_t tick = lancBitTick;           //

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

// ─── LANC bit-bang (polling, called when a LANC command is pending) ───────────
// Sony LANC: idle HIGH, start bit LOW (1.5 ms), 8 data bits LSB-first at 9600 baud
// Bit period ≈ 104 µs
#define LANC_BIT_US  104

void sendLancByte(uint8_t cmdPin, uint8_t b) {
  // Start bit
  digitalWrite(cmdPin, LOW);
  delayMicroseconds(LANC_BIT_US);
  // 8 data bits, LSB first
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(cmdPin, (b >> i) & 0x01 ? HIGH : LOW);
    delayMicroseconds(LANC_BIT_US);
  }
  // Stop bit
  digitalWrite(cmdPin, HIGH);
  delayMicroseconds(LANC_BIT_US);
}

void dispatchLancCommand() {
  if (activeCam < 0 || !lancCmdPending) return;
  uint8_t cp = lancCmdPin[activeCam];
  pinMode(cp, OUTPUT);
  digitalWrite(cp, HIGH);   // idle high
  sendLancByte(cp, lancCmdByte);
  if (lancCmdByte2 != 0x00) {
    sendLancByte(cp, lancCmdByte2);
  }
  lancCmdPending = false;
}

// ═══════════════════════════════════════════════════════════════════════════════
// COMMAND PARSING ROUTINE
// Call from loop(); accumulates chars, acts on complete '<...>' frames.
//
// Supported frame formats:
//   <CamId D>            select camera + pan/tilt:  e.g. <1U> <2L> <3S>
//   <CamId Z B1 B2>      raw LANC, two hex bytes:   e.g. <1Z 28 00>
//   (spaces in LANC frame are optional / ignored)
// ═══════════════════════════════════════════════════════════════════════════════
void parseSerialInput() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (!inFrame) {
      if (c == STX) {
        inFrame = true;
        rxLen   = 0;
        memset(rxBuf, 0, sizeof(rxBuf));
        Serial.println(rxAckSTX);   // ack start marker
      }
      // anything before STX is silently ignored
      continue;
    }

    // Inside frame
    if (c == ETX) {
      inFrame = false;
      Serial.println(rxAckETX);   // ack end marker
      processFrame(rxBuf, rxLen);
      rxLen = 0;
      continue;
    }

    // Buffer overflow guard
    if (rxLen < RX_BUF_SIZE - 1) {
      rxBuf[rxLen++] = c;
      rxBuf[rxLen]   = '\0';
      Serial.println(rxAckCh);    // ack each character received
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

  if (camIdx < 0) {
    Serial.println("ERR:CAMID");
    return;
  }

  // Ack camera ID
  Serial.print(rxAckCID);
  Serial.print('\r');
  Serial.println();

  // Switch active camera if changed; release pan/tilt of previous
  if (camIdx != activeCam) {
    if (activeCam >= 0) releasePanTilt((uint8_t)activeCam);
    activeCam = camIdx;
  }

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

    case CMD_STOP:
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
        if (hIdx < 4) { Serial.println("ERR:LANC_DATA"); break; }

        uint8_t b1 = (uint8_t)strtol((char[]){hexStr[0], hexStr[1], '\0'}, NULL, 16);
        uint8_t b2 = (uint8_t)strtol((char[]){hexStr[2], hexStr[3], '\0'}, NULL, 16);
        queueLancCommand(b1, b2, 2);
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
  lancCmdOutPin       = lancCmdPin[activeCam];
  lancPendingBuf[0]   = b1;
  lancPendingBuf[1]   = b2;
  lancPendingLen      = len;
  lancCmdPending      = true;   // ISR picks this up on next tick
}

// ═══════════════════════════════════════════════════════════════════════════════
// loop()
// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
  // 1. Service incoming USB-serial frames from host
  parseSerialInput();

  // 2. If host disconnects (DTR dropped), reset state and wait for reconnect
  if (hostConnected && !Serial) {
    hostConnected = false;
    activeCam     = -1;
    releaseAllPanTilt();
    Serial.println("DISCONNECTED");
    // Spin until host reconnects and sends '!'
    while (true) {
      if (Serial && Serial.available()) {
        char c = (char)Serial.read();
        if (c == HostListeningCode) {
          hostConnected = true;
          Serial.println("Arduino LANC to USB-serial interface v1.0");
          break;
        }
      }
    }
  }

  // 3. No other blocking work here — LANC TX is handled entirely by lancTimerISR,
  //    and pan/tilt pins are set directly inside processFrame().
}

// ═══════════════════════════════════════════════════════════════════════════════
// INIT ROUTINE
// ═══════════════════════════════════════════════════════════════════════════════
void initHardware() {
  // --- Serial (USB virtual COM) ---
  Serial.begin(115200);
  // Wait for host to open port (native USB)
  uint32_t t = millis();
  while (!Serial && (millis() - t < 5000));

  // --- Pan/tilt pins: all HIGH-Z by default ---
  releaseAllPanTilt();

  // --- LANC CMD pins: OUTPUT, idle HIGH ---
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(lancCmdPin[i], OUTPUT);
    digitalWrite(lancCmdPin[i], HIGH);
  }

  // --- LANC SIG_IN pins: INPUT, pull-up (LANC bus is active-low) ---
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(lancSigPin[i], INPUT_PULLUP);
  }

  // --- LANC timer: AGT0 fires every LANC_HALF_BIT_US (52 µs) ---
  // Use AGT periodic timer channel 0
  if (lancTimer.begin(TIMER_MODE_PERIODIC,
                      AGT_TIMER,
                      0,                        // channel 0
                      1.0e6f / LANC_HALF_BIT_US, // frequency in Hz (~19231 Hz)
                      0.0f,
                      lancTimerISR)) {
    lancTimer.setup_overflow_irq();
    lancTimer.open();
    lancTimer.start();
  }

  // --- Application state ---
  activeCam     = -1;
  hostConnected = false;
  inFrame       = false;
  rxLen         = 0;

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

