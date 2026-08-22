/*
 * Arduino UNO R4 WiFi — USB-Serial to LANC + Pan/Tilt Interface v1.0
 *
 * Pan/Tilt outputs: default HIGH-Z (INPUT), driven LOW when actuated.
 * LANC: bidirectional split into CMD_OUT (output) and SIG_IN (input).
 * Only one LANC port active at a time.
 * Serial handshake uses '<' '>' framing; host announces with '!'.
 */

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

// ─── Pan/Tilt pin tables (indexed 0=CAM1, 1=CAM2, 2=CAM3) ───────────────────
const uint8_t panLeftPin[3]  = { CAM1_PAN_LEFT,  CAM2_PAN_LEFT,  CAM3_PAN_LEFT  };
const uint8_t panRightPin[3] = { CAM1_PAN_RIGHT, CAM2_PAN_RIGHT, CAM3_PAN_RIGHT };
const uint8_t tiltUpPin[3]   = { CAM1_TILT_UP,   CAM2_TILT_UP,   CAM3_TILT_UP   };
const uint8_t tiltDownPin[3] = { CAM1_TILT_DOWN, CAM2_TILT_DOWN, CAM3_TILT_DOWN };

// All pan/tilt pins flat list for easy init
const uint8_t allPanTiltPins[] = {
  CAM1_PAN_LEFT, CAM1_PAN_RIGHT, CAM1_TILT_UP, CAM1_TILT_DOWN,
  CAM2_PAN_LEFT, CAM2_PAN_RIGHT, CAM2_TILT_UP, CAM2_TILT_DOWN,
  CAM3_PAN_LEFT, CAM3_PAN_RIGHT, CAM3_TILT_UP, CAM3_TILT_DOWN
};

// ─── LANC pin tables ──────────────────────────────────────────────────────────
const uint8_t lancCmdPin[3] = { CAM1_LANC_CMD_OUT, CAM2_LANC_CMD_OUT, CAM3_LANC_CMD_OUT };
const uint8_t lancSigPin[3] = { CAM1_LANC_SIG_IN,  CAM2_LANC_SIG_IN,  CAM3_LANC_SIG_IN  };

// ─── State ───────────────────────────────────────────────────────────────────
int8_t  activeCam      = -1;   // 0-based; -1 = none selected
bool    hostConnected  = false;

// ─── LANC frame state (interrupt-driven, one camera active) ──────────────────
volatile uint8_t  lancCmdByte    = 0x00;   // command byte to transmit
volatile uint8_t  lancCmdByte2   = 0x00;   // second LANC byte (if needed)
volatile bool     lancCmdPending = false;

// ─── Helpers ─────────────────────────────────────────────────────────────────

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

// ─── Message parsing ──────────────────────────────────────────────────────────
// Expected frame: <CamId CommandChar> e.g. <1U>
// Extended:       <CamId CommandChar DataChar> for future LANC codes