/*
Arduino Nano Every LANC to USB-serial interface by Novgorod
Version 1.0
Github: 

Based on Arduino code by L. Rosén: https://projecthub.arduino.cc/L-Rosen/9b5d02d4-f885-41ee-bba7-6b18d3dfe47d
LANC protocol: http://www.boehmel.de/lanc.htm
*/

// Use ATMEGA4809 native registers (direct I/O) on Arduino Nano Every!

// GLOBAL
#define ledON (VPORTE.OUT = VPORTE.IN |= B00000100)      // Set digital pin 13 (PE2)
#define ledOFF (VPORTE.OUT = VPORTE.IN |= B11111011)     // Clear digital pin 13 (PE2)
#define lancRepeats 25                                   // Repeat LANC command (4 frames default)
#define dbgPinON (VPORTE.OUT = VPORTE.IN |= B00000010)   // Set digital pin 12 (PE1)
#define dbgPinOFF (VPORTE.OUT = VPORTE.IN &= B11111101)  // Clear digital pin 12 (PE1)
#define rxAckSTX "FE"                                    // "Received Start-Of-Text code"
#define rxAckCh "AA"                                     // "Received Character code"
#define rxAckETX "EF"                                    // "Received End-Of-Text code"
#define rxAckCID '@'
#define rxAckCmd "BB"
#define rxAckDir "CC"
#define rxAckOnOff "DD"
#define rxAck
#define camUnknown '0'
#define cam1Id '1'
#define cam2Id '2'
#define cam3Id '3'
#define cam1PT '4'
#define cam2PT '5'
#define cam3PT '6'
#define pan 'A'
#define tilt 'B'
#define up_left 'C'
#define down_right 'D'
#define ON '1'
#define OFF '0'


// CAMERA 1
#define cam1CmdPinON (VPORTA.OUT = VPORTA.IN |= B00000001)   // Set digital pin 2 (PA0)
#define cam1CmdPinOFF (VPORTA.OUT = VPORTA.IN &= B11111110)  // Clear digital pin 2 (PA0)
#define cam1LancPinREAD (VPORTF.IN &= B00100000)             // Read digital pin 3 (PF5)
#define cam1UpPinON (VPORTE.OUT = VPORTE.IN |= B00000001)    // Set digital pin 11 (PE0)
#define cam1UpPinOFF (VPORTE.OUT = VPORTE.IN &= B11111110)   // Clear digital pin 11 (PE0)
#define cam1DnPinON (VPORTB.OUT = VPORTB.IN |= B00000010)    // Set digital pin 10 (PB1)
#define cam1DnPinOFF (VPORTB.OUT = VPORTB.IN &= B11111101)   // Clear digital pin 10 (PB1)
#define cam1RPinON (VPORTB.OUT = VPORTB.IN |= B00000001)     // Set digital pin 9 (PB0)
#define cam1RPinOFF (VPORTB.OUT = VPORTB.IN &= B11111110)    // Clear digital pin 9 (PB0)
#define cam1LPinON (VPORTE.OUT = VPORTE.IN |= B00001000)     // Set digital pin 8 (PE3)
#define cam1LPinOFF (VPORTE.OUT = VPORTE.IN &= B11110111)    // Clear digital pin 8 (PE3)

// CAMERA 2
#define cam2CmdPinON (VPORTC.OUT = VPORTC.IN |= B01000000)   // Set digital pin 4 (PC6)
#define cam2CmdPinOFF (VPORTC.OUT = VPORTC.IN &= B10111111)  // Clear digital pin 4 (PC6)
#define cam2LancPinREAD (VPORTB.IN &= B00000100)             // Read digital pin 5 (PB2)
#define cam2UpPinON (VPORTD.OUT = VPORTD.IN |= B00001000)    // Set digital pin 14 (PD3)
#define cam2UpPinOFF (VPORTD.OUT = VPORTD.IN &= B11110111)   // Clear digital pin 14 (PD3)
#define cam2DnPinON (VPORTD.OUT = VPORTD.IN |= B00000100)    // Set digital pin 15 (PD2)
#define cam2DnPinOFF (VPORTD.OUT = VPORTD.IN &= B11111011)   // Clear digital pin 15 (PD2)
#define cam2RPinON (VPORTD.OUT = VPORTD.IN |= B00000010)     // Set digital pin 16 (PD1)
#define cam2RPinOFF (VPORTD.OUT = VPORTD.IN &= B11111101)    // Clear digital pin 16 (PD1)
#define cam2LPinON (VPORTD.OUT = VPORTD.IN |= B00000001)     // Set digital pin 17 (PD0)
#define cam2LPinOFF (VPORTD.OUT = VPORTD.IN &= B11111110)    // Clear digital pin 17 (PD0)

// CAMERA 3
#define cam3CmdPinON (VPORTF.OUT = VPORTF.IN |= B00010000)   // Set digital pin 6 (PF4)
#define cam3CmdPinOFF (VPORTF.OUT = VPORTF.IN &= B11101111)  // Clear digital pin 6 (PF4)
#define cam3LancPinREAD (VPORTA.IN &= B00000010)             // Read digital pin 7 (PA1)
#define cam3UpPinON (VPORTA.OUT = VPORTA.IN |= B00000100)    // Set digital pin 18 (PA2)
#define cam3UpPinOFF (VPORTA.OUT = VPORTA.IN &= B11111011)   // Clear digital pin 18 (PA2)
#define cam3DnPinON (VPORTA.OUT = VPORTA.IN |= B00001000)    // Set digital pin 19 (PA3)
#define cam3DnPinOFF (VPORTA.OUT = VPORTA.IN &= B11110111)   // Clear digital pin 19 (PA3)
#define cam3RPinON (VPORTD.OUT = VPORTD.IN |= B00010000)     // Set digital pin 20 (PD4)
#define cam3RPinOFF (VPORTD.OUT = VPORTD.IN &= B11101111)    // Clear digital pin 20 (PD4)
#define cam3LPinON (VPORTD.OUT = VPORTA.IN |= B00100000)     // Set digital pin 21 (PD5)
#define cam3LPinOFF (VPORTD.OUT = VPORTA.IN &= B11011111)    // Clear digital pin 21 (PD5)

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

int bitDura = 104;     // Duration of one LANC bit in microseconds, 104µs -> 9600bps
int halfbitDura = 52;  // Half bit duration
int repeats = 0;
byte lancByte = 0;
byte strPointer = 0;
char inChar;
boolean strComplete = false;
boolean lancCmd[16];
unsigned long time;
const byte numChars = 5;
char inString[numChars];
boolean newData = false;
char currCam = camUnknown;  // cam ID until ID is received
char currCmd = pan;         // Command for pan/tilt operations
char currDir = up_left;
char currOnOff = OFF;
boolean recvInProgress = false;
boolean gotCamId = false;

void setup() {
  VPORTE.DIR |= B00000100;  // Config ledPin as output (high)
  VPORTE.DIR |= B00000010;  // Config dbgPin as output (high)
  dbgPinOFF;                // Clear debug pin

  // CAMERA 1
  VPORTA.DIR |= B00000001;  // Config cam1CmdPin as output (high)
  cam1CmdPinOFF;            // Clear Camera 1 LANC control pin (LANC line becomes high)
  VPORTF.DIR &= B11011111;  // Config cam1LancPin as input (low)
  VPORTE.DIR |= B00000001;  // Config cam1UpPin as output (high)
  cam1UpPinOFF;             // Clear Camera 1 UP control pin
  VPORTB.DIR |= B00000010;  // Config cam1DnPin as output (high)
  cam1DnPinOFF;             // Clear Camera 1 DN control pin
  VPORTB.DIR |= B00000001;  // Config cam1RPin as output (high)
  cam1RPinOFF;              // Clear Camera 1 R control pin
  VPORTE.DIR |= B00001000;  // Config cam1LPin as output (high)
  cam1LPinOFF;              // Clear Camera 1 L control pin

  // CAMERA 2
  VPORTC.DIR |= B00010000;  // Config cam2CmdPin as output (high)
  cam2CmdPinOFF;            // Clear Camera 2 LANC control pin (LANC line becomes high)
  VPORTB.DIR &= B11111011;  // Config cam2LancPin as input (low)
  VPORTD.DIR |= B00001000;  // Config cam2UpPin as output (high)
  cam2UpPinOFF;             // Clear Camera 2 UP control pin
  VPORTD.DIR |= B00000100;  // Config cam2DnPin as output (high)
  cam2DnPinOFF;             // Clear Camera 2 DN control pin
  VPORTD.DIR |= B00000010;  // Config cam2RPin as output (high)
  cam2RPinOFF;              // Clear Camera 2 R control pin
  VPORTD.DIR |= B00000001;  // Config cam2LPin as output (high)
  cam2LPinOFF;              // Clear Camera 2 L control pin

  // CAMERA 3
  VPORTF.DIR |= B00010000;  // Config cam3CmdPin as output (high)
  cam3CmdPinOFF;            // Clear Camera 3 LANC control pin (LANC line becomes high)
  VPORTA.DIR &= B11111101;  // Config cam3LancPin as input (low)
  VPORTA.DIR |= B00000100;  // Config cam3UpPin as output (high)
  cam3UpPinOFF;             // Clear Camera 3 UP control pin
  VPORTA.DIR |= B00001000;  // Config cam3DnPin as output (high)
  cam3DnPinOFF;             // Clear Camera 3 DN control pin
  VPORTD.DIR |= B00010000;  // Config cam3RPin as output (high)
  cam3RPinOFF;              // Clear Camera 3 R control pin
  VPORTD.DIR |= B00100000;  // Config cam3LPin as output (high)
  cam3LPinOFF;              // Clear Camera 3 L control pin

  Serial.begin(115200);  // Start serial port
  //Serial1.begin(115200);      // Bridge port to header

  if (recvHostListeningCode()) {
    Serial.println("Arduino LANC to USB-serial interface v1.0");
  }
}
boolean recvHostListeningCode() {
  char HostListeningCode = '!';
  char rc;
  while (!(Serial.available())) {};
  rc = Serial.read();
  //Serial1.write(rc); //Send RX0 byte to TX1
  return (rc == HostListeningCode) ? 1 : 0;
}

void camUnknownProc() {
  recvUpToCamId();
  newData = false;
}

void cam1Proc() {
  time = micros();  // Wait for lancPin to be high for at least 5ms
  while (micros() - time < 5000) {
    if (!cam1LancPinREAD) { time = micros(); }
  }

  noInterrupts();  // Disable interrupts for time-critical jitter-free bit-banging

  while (cam1LancPinREAD) {}  // Wait for the falling edge indicating the begin of the start bit

  ledON;  // LED indicator on = LANC message start
  //  if(repeats != 0) {
  //   Serial.println(repeats); }
  for (int bytenr = 0; bytenr < 8; bytenr++) {  // Process 8-byte frame
    delayMicroseconds(bitDura - 4);             // Wait start bit duration at the beginning of a byte
    for (int bitnr = 0; bitnr < 8; bitnr++) {   // Process 8 bits
      if (bytenr < 2 && repeats) {              // Output data (if available) during the first two bytes
        if (lancCmd[bitnr + bytenr * 8]) {
          cam1CmdPinON;
        } else {
          cam1CmdPinOFF;
        }
      }
      delayMicroseconds(halfbitDura - 3);
      bitWrite(lancByte, bitnr, !cam1LancPinREAD);  // Read data line during middle of bit and write the bit to lancByte (LANC is inverted!)
      delayMicroseconds(halfbitDura);
    }
    cam1CmdPinOFF;
    delayMicroseconds(halfbitDura - 10);  // Make sure to be in the stop bit before waiting for next byte; small delay adjust for sending serial data
    //Serial.write(lancByte);                         // Send lancByte through serial port while waiting for next start bit
    if (bytenr < 7) {
      while (cam1LancPinREAD) {}
    }  // Wait as long as the LANC line is high until the next start bit EXCEPT at end of frame
  }

  //Serial.write(10);                               // Write line feed (0xA) to serial port after frame

  if (repeats > 0) { repeats--; }  // If a LANC command was sent in this frame, decrease the send "queue"; nothing is sent if repeats is 0

  ledOFF;        // LED indicator on = LANC message end
  interrupts();  // Re-enable interrupts

  recvWithStartEndMarkers();
  newData = false;
}

void cam2Proc() {
  time = micros();  // Wait for lancPin to be high for at least 5ms
  while (micros() - time < 5000) {
    if (!cam2LancPinREAD) { time = micros(); }
  }

  noInterrupts();  // Disable interrupts for time-critical jitter-free bit-banging

  while (cam2LancPinREAD) {}  // Wait for the falling edge indicating the begin of the start bit

  ledON;                                        // LED indicator on = LANC message start
                                                //  if(repeats != 0) {
                                                //   Serial.println(repeats); }
  for (int bytenr = 0; bytenr < 8; bytenr++) {  // Process 8-byte frame
    delayMicroseconds(bitDura - 4);             // Wait start bit duration at the beginning of a byte
    for (int bitnr = 0; bitnr < 8; bitnr++) {   // Process 8 bits
      if (bytenr < 2 && repeats) {              // Output data (if available) during the first two bytes
        if (lancCmd[bitnr + bytenr * 8]) {
          cam2CmdPinON;
        } else {
          cam2CmdPinOFF;
        }
      }
      delayMicroseconds(halfbitDura - 3);
      bitWrite(lancByte, bitnr, !cam2LancPinREAD);  // Read data line during middle of bit and write the bit to lancByte (LANC is inverted!)
      delayMicroseconds(halfbitDura);
    }
    cam2CmdPinOFF;
    delayMicroseconds(halfbitDura - 10);  // Make sure to be in the stop bit before waiting for next byte; small delay adjust for sending serial data
    //Serial.write(lancByte);                         // Send lancByte through serial port while waiting for next start bit
    if (bytenr < 7) {
      while (cam2LancPinREAD) {}
    }  // Wait as long as the LANC line is high until the next start bit EXCEPT at end of frame
  }

  //Serial.write(10);                               // Write line feed (0xA) to serial port after frame

  if (repeats > 0) { repeats--; }  // If a LANC command was sent in this frame, decrease the send "queue"; nothing is sent if repeats is 0

  ledOFF;        // LED indicator on = LANC message end
  interrupts();  // Re-enable interrupts

  recvWithStartEndMarkers();
  newData = false;
}

void cam3Proc() {
  time = micros();  // Wait for lancPin to be high for at least 5ms
  while (micros() - time < 5000) {
    if (!cam3LancPinREAD) { time = micros(); }
  }

  noInterrupts();  // Disable interrupts for time-critical jitter-free bit-banging

  while (cam3LancPinREAD) {}  // Wait for the falling edge indicating the begin of the start bit


  ledON;                                        // LED indicator on = LANC message start
                                                //  if(repeats != 0) {
                                                //   Serial.println(repeats); }
  for (int bytenr = 0; bytenr < 8; bytenr++) {  // Process 8-byte frame
    delayMicroseconds(bitDura - 4);             // Wait start bit duration at the beginning of a byte
    for (int bitnr = 0; bitnr < 8; bitnr++) {   // Process 8 bits
      if (bytenr < 2 && repeats) {              // Output data (if available) during the first two bytes
        if (lancCmd[bitnr + bytenr * 8]) {
          cam3CmdPinON;
        } else {
          cam3CmdPinOFF;
        }
      }
      delayMicroseconds(halfbitDura - 3);
      bitWrite(lancByte, bitnr, !cam3LancPinREAD);  // Read data line during middle of bit and write the bit to lancByte (LANC is inverted!)
      delayMicroseconds(halfbitDura);
    }
    cam3CmdPinOFF;
    delayMicroseconds(halfbitDura - 10);  // Make sure to be in the stop bit before waiting for next byte; small delay adjust for sending serial data
    //Serial.write(lancByte);                         // Send lancByte through serial port while waiting for next start bit
    if (bytenr < 7) {
      while (cam3LancPinREAD) {}
    }  // Wait as long as the LANC line is high until the next start bit EXCEPT at end of frame
  }

  //Serial.write(10);                               // Write line feed (0xA) to serial port after frame

  if (repeats > 0) { repeats--; }  // If a LANC command was sent in this frame, decrease the send "queue"; nothing is sent if repeats is 0

  ledOFF;        // LED indicator on = LANC message end
  interrupts();  // Re-enable interrupts

  recvWithStartEndMarkers();
  newData = false;
}

void cam1PtProc() {  // similar to zoom proc, but no data sent to camera over LANC
  recvPtudWithStartEndMarkers();
  newData = false;
}

void cam2PtProc() {  // similar to zoom proc, but no data sent to camera over LANC
  recvPtudWithStartEndMarkers();
  newData = false;
}

void cam3PtProc() {  // similar to zoom proc, but no data sent to camera over LANC
  recvPtudWithStartEndMarkers();
  newData = false;
}

void loop() {
  // Execution loops through the basic receive function until the camera ID is received
  switch (currCam) {
    case camUnknown:
      camUnknownProc(); // receive up to Cam ID
      break; 
    case cam1Id : cam1Proc();
      break;
    case cam2Id:
      cam2Proc();
      break;
    case cam3Id:
      cam3Proc();
      break;
    case cam1PT:
      cam1Proc();
      break;
    case cam2PT:
      cam2Proc();
      break;
    case cam3PT:
      cam3Proc();
      break;
    default:
      camUnkownProc();
  }
}

boolean recvUpToCamId() {
  char startMarker = '<';
  char rc;
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (recvInProgress == true) {
      if (gotCamId == false) {  // if we haven't yet picked up the camera ID, get it now...
        switch (rc) {
          case cam1:
            currCam = cam1ID;
            break;
          case cam2:
            currCam = cam2ID;
            break;
          case cam3:
            currCam = cam3ID;
            break;
          case cam1PT:
            currCam = cam1PT;
            break;
          case cam2PT:
            currCam = cam2PT;
            break;
          case cam3PT:
            currCam = cam3PT;
            break;
          default:
            currCam = cam1;  // if the char is not a cam ID, make it cam1...
        }
        currCam = rc;
        gotCamId = true;

        Serial.println(rxAckCID);

      } else if (rc == startMarker) {
        recvInProgress = true;
        Serial.println(rxAckSTX);
      }
    }
  }

  boolean recvRemaining() {
  }

  boolean recvRemainingPT() {
  }

  boolean recvPtudWithStartEndMarkers() {

    static boolean recvInProgress = false;
    static boolean gotCamId = false;
    static boolean gotCmd = false;
    static boolean gotDir = false;
    static boolean gotOnOff = false;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
      rc = Serial.read();

      if (recvInProgress == true) {
        // receive data characters
        if (rc != endMarker) {
          if (gotCamId == false) {  // if we haven't yet picked up the camera ID, get it now...
            switch (rc) {
              case cam1PT:
                currCam = cam1PT;
                break;
              case cam2PT:
                currCam = cam2PT;
                break;
              case cam3PT:
                currCam = cam3PT;
                break;
              default:
                currCam = cam1PT;  // if the char is not a cam ID, make it cam1...
            }
            currCam = rc;
            gotCamId = true;

            Serial.println(rxAckCID);
          } else if (gotCmd == false) {  // if we haven't yet picked up the command ID, get it now...

            switch (rc) {
              case pan:
                currCmd = pan;
                break;
              case tilt:
                currCmd = tilt;
                break;
              default:
                currCmd = pan;  // if the char is not a command, make it pan...
            }
            gotCmd = true;
            Serial.println(rxAckCmd);
          } else if (gotDir == false) {  // if we haven't yet picked up the direction, get it now...

            switch (rc) {
              case up_down:
                currCmd = up_left;
                break;
              case left_right:
                currCmd = down_right;
                break;
              default:
                currCmd = up_left;  // if the char is not a direction, make it up_left...
            }
            gotDir = true;
            Serial.println(rxAckDir);
          } else if (gotOnOff == false) {  // if we haven't yet picked up whether it's ON or OFF, get it now...

            switch (rc) {
              case ON:
                currOnOff = ON;
                break;
              case OFF:
                currOnOff = OFF;
                break;
              default:
                currOnOff = OFF;  // if the char is not ON or OFF, make it OFF...
            }
            gotOnOff = true;
            Serial.println(rxAckOnOff);
          }
        } else {
          // received end marker
          // re-initialize variables
          recvInProgress = false;
          gotCamId = false;
          gotCmd = flase;
          gotDir = false;
          gotOnOff = false ndx = 0;
          newData = true;
          Serial.println(rxAckETX);
          // Serial buffer flushed:
          flushSerialBuffer();

          return (1);
        }
      }

      else if (rc == startMarker) {
        recvInProgress = true;
        Serial.println(rxAckSTX);
      }
    }
  }

  boolean recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static boolean gotCamId = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
      rc = Serial.read();

      if (recvInProgress == true) {
        // receive data characters
        if (gotCamId == false) {  // if we haven't yet picked up the camera ID, get it now...
          switch (rc) {
            case cam1Id:
              currCam = cam1Id;
              break;
            case cam2Id:
              currCam = cam2Id;
              break;
            case cam3Id:
              currCam = cam3Id;
              break;
            default:
              currCam = cam1Id;  // if the char is not a cam ID, make it cam1...
          }
          gotCamId = true;
          Serial.println(rxAckCID);
        } else {  // already have the camera ID, everything else is data or end marker...
          if (rc != endMarker) {

            Serial.println(rxAckCh);
          } else {
            // received end marker
            // re-initialize variables
            recvInProgress = false;
            gotCamId = false;
            ndx = 0;
            newData = true;
            Serial.println(rxAckETX);
            if (hexchartobitarray()) {
              repeats = lancRepeats;
            }                                                        // Convert input string and set LANC commands "queue" (number of frames to repeat command)
            for (int i = 0; i < numChars; i++) { inString[i] = 0; }  // Reset input string (optional cleanup)

            // Serial buffer flushed:
            flushSerialBuffer();

            return (1);
          }
        }
      } else if (rc == startMarker) {
        recvInProgress = true;
        Serial.println(rxAckSTX);
      }
    }
  }

  void flushSerialBuffer() {
    // flush the serial input buffer
    while (Serial.available() > 0) {
      Serial.read();
    }
  }

  boolean hexchartobitarray() {
    // This function converts the hex char LANC command and fills the lancCmd array with the bits in LSB-first order

    int byte1, byte2;



    for (int i = 0; i < 4; i++) {
      if (!(isHexadecimalDigit(inString[i]))) {
        return 0;
      }
    }

    byte1 = (hexchartoint(inString[0]) << 4) + hexchartoint(inString[1]);
    byte2 = (hexchartoint(inString[2]) << 4) + hexchartoint(inString[3]);

    for (int i = 0; i < 8; i++) { lancCmd[i] = bitRead(byte1, i); }
    for (int i = 0; i < 8; i++) { lancCmd[i + 8] = bitRead(byte2, i); }

    return 1;
  }

  int hexchartoint(char hexchar) {
    switch (hexchar) {
      case 'F':
        return 15;
        break;
      case 'f':
        return 15;
        break;
      case 'E':
        return 14;
        break;
      case 'e':
        return 14;
        break;
      case 'D':
        return 13;
        break;
      case 'd':
        return 13;
        break;
      case 'C':
        return 12;
        break;
      case 'c':
        return 12;
        break;
      case 'B':
        return 11;
        break;
      case 'b':
        return 11;
        break;
      case 'A':
        return 10;
        break;
      case 'a':
        return 10;
        break;
      default:
        return (int)(hexchar - 48);
        break;
    }
  }