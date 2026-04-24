#include <Arduino.h>

const byte dccPin = 2;

// ============================================================
// Konfiguration
// ============================================================

// -1 = alle Lokpakete
volatile int TARGET_ADDR = -1;

// Rohpakete zusätzlich ausgeben?
const bool DEBUG_RAW = true;

// ============================================================
// DCC Rohdecoder (funktionierende Basis)
// ============================================================

volatile unsigned long lastEdge = 0;
volatile int pendingHalf = -1;      // -1 = none, 0 = long, 1 = short

volatile byte packet[8];
volatile byte packetLen = 0;
volatile byte curByte = 0;
volatile byte bitCount = 0;
volatile byte preambleCount = 0;
volatile bool inPacket = false;
volatile bool packetReady = false;

void resetDecoder() {
  pendingHalf = -1;
  packetLen = 0;
  curByte = 0;
  bitCount = 0;
  preambleCount = 0;
  inPacket = false;
}

void finishPacket() {
  packetReady = true;
  inPacket = false;
  curByte = 0;
  bitCount = 0;
  preambleCount = 1; // End-1 kann schon Teil der nächsten Präambel sein
}

void pushBit(byte b) {
  if (!inPacket) {
    if (b == 1) {
      preambleCount++;
    } else {
      if (preambleCount >= 10) {
        inPacket = true;
        packetLen = 0;
        curByte = 0;
        bitCount = 0;
      }
      preambleCount = 0;
    }
    return;
  }

  if (bitCount < 8) {
    curByte = (curByte << 1) | b;
    bitCount++;
    return;
  }

  // 9. Bit = Separator / Endbit
  if (packetLen < sizeof(packet)) {
    packet[packetLen++] = curByte;
  } else {
    resetDecoder();
    return;
  }

  curByte = 0;
  bitCount = 0;

  if (b == 1) {
    finishPacket();   // Paketende
  }
  // bei b == 0 folgt nächstes Byte
}

void dccISR() {
  unsigned long now = micros();
  unsigned long dt = now - lastEdge;
  lastEdge = now;

  int halfType = -1;

  // an deine gemessenen Zeiten angepasst
  if (dt >= 44 && dt <= 68) {
    halfType = 1;   // kurze Halbwelle = DCC "1"
  } else if (dt >= 90 && dt <= 700) {
    halfType = 0;   // lange Halbwelle = DCC "0"
  } else {
    pendingHalf = -1;   // resync
    return;
  }

  if (pendingHalf == -1) {
    pendingHalf = halfType;
    return;
  }

  // Ein DCC-Bit besteht aus zwei Halbwellen gleicher Klasse
  if (pendingHalf == halfType) {
    pushBit((byte)halfType);
  }

  pendingHalf = -1;
}

// ============================================================
// Lokstatus
// ============================================================

int  curAddr = -1;
bool curAddrIsLong = false;

int  curSpeed = -1;     // -1 unbekannt, 0 STOP, >0 Fahrstufe
bool curDir = true;     // true = vorwärts
bool curFunc[29];       // F0..F28

int  prevAddr = -1;
bool prevAddrIsLong = false;
int  prevSpeed = -1;
bool prevDir = true;
bool prevFunc[29];

void resetState() {
  curAddr = -1;
  curAddrIsLong = false;
  prevAddr = -1;
  prevAddrIsLong = false;

  curSpeed = -1;
  prevSpeed = -1;
  curDir = true;
  prevDir = true;

  for (int i = 0; i < 29; i++) {
    curFunc[i] = false;
    prevFunc[i] = false;
  }
}

void printFunctions() {
  Serial.print("funktionen: [");
  for (int i = 0; i < 29; i++) {
    if (i) Serial.print(", ");
    Serial.print(curFunc[i] ? "true" : "false");
  }
  Serial.println("]");
}

void printState(const char* typ) {
  Serial.print("typ: ");
  Serial.println(typ);

  Serial.print("adresse: ");
  Serial.println(curAddr);

  Serial.print("adressmodus: ");
  Serial.println(curAddrIsLong ? "long" : "short");

  Serial.print("fahrstufe: ");
  Serial.println(curSpeed);

  Serial.print("richtung: ");
  Serial.println(curDir ? "true" : "false");

  printFunctions();
  Serial.println();
}

void snapshotState() {
  prevAddr = curAddr;
  prevAddrIsLong = curAddrIsLong;
  prevSpeed = curSpeed;
  prevDir = curDir;
  for (int i = 0; i < 29; i++) prevFunc[i] = curFunc[i];
}

void maybePrint(const char* typ, bool changedSpeed, bool changedFunc) {
  if (!changedSpeed && !changedFunc) return;
  printState(typ);
  snapshotState();
}

// ============================================================
// Serial
// ============================================================

void handleSerial() {
  if (!Serial.available()) return;

  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() == 0) return;

  if (s.equalsIgnoreCase("ALL")) {
    TARGET_ADDR = -1;
    resetState();
    Serial.println("Filter: ALL");
    return;
  }

  if (s.charAt(0) == 'S' || s.charAt(0) == 's') {
    int adr = s.substring(1).toInt();
    if (adr >= 1 && adr <= 127) {
      TARGET_ADDR = adr;
      resetState();
      Serial.print("Filter kurze Adresse: ");
      Serial.println(TARGET_ADDR);
    } else {
      Serial.println("Kurzadresse muss 1..127 sein");
    }
    return;
  }

  if (s.charAt(0) == 'L' || s.charAt(0) == 'l') {
    int adr = s.substring(1).toInt();
    if (adr >= 128 && adr <= 10239) {
      TARGET_ADDR = adr;
      resetState();
      Serial.print("Filter lange Adresse: ");
      Serial.println(TARGET_ADDR);
    } else {
      Serial.println("Langadresse muss 128..10239 sein");
    }
    return;
  }

  Serial.println("Kommandos: ALL, S<num>, L<num>");
}

// ============================================================
// Hilfen Paket / Adresse
// ============================================================

bool packetXorOk(const byte* local, byte len) {
  if (len < 3) return false;
  byte x = 0;
  for (byte i = 0; i < len - 1; i++) x ^= local[i];
  return x == local[len - 1];
}

bool decodeLocoAddress(const byte* local, byte len, int& adr, bool& isLong, byte& instrIndex) {
  adr = -1;
  isLong = false;
  instrIndex = 1;

  if (len < 3) return false;

  // Idle ignorieren
  if (local[0] == 0xFF && local[1] == 0x00 && local[len - 1] == 0xFF) return false;

  // kurze Adresse: 0xxxxxxx
  if ((local[0] & 0x80) == 0) {
    adr = local[0];
    if (adr == 0 || adr == 0xFF) return false;
    isLong = false;
    instrIndex = 1;
    return true;
  }

  // lange Adresse: 11aaaaaa aaaaaaaa
  if ((local[0] & 0xC0) == 0xC0 && len >= 4) {
    adr = ((int)(local[0] & 0x3F) << 8) | local[1];
    isLong = true;
    instrIndex = 2;
    return true;
  }

  return false;
}

void printRawPacket(int adr, bool isLong, const byte* local, byte len, byte instrIndex) {
  if (!DEBUG_RAW) return;

  Serial.print("RAW adr=");
  Serial.print(adr);
  Serial.print(isLong ? " (lang)" : " (kurz)");
  Serial.print(" | Paket: ");
  for (byte i = 0; i < len; i++) {
    if (local[i] < 16) Serial.print('0');
    Serial.print(local[i], HEX);
    Serial.print(' ');
  }
  Serial.print("| Instr: 0x");
  if (local[instrIndex] < 16) Serial.print('0');
  Serial.print(local[instrIndex], HEX);
  Serial.println();
}

// ============================================================
// DCC Befehle dekodieren
// ============================================================

void applyAddressContext(int adr, bool isLong) {
  curAddr = adr;
  curAddrIsLong = isLong;
}

void decodeFunctions_F0_F4(byte instr, bool& changedFunc) {
  // 100DDDDD, davon:
  // bit4 = F0, bit0..3 = F1..F4
  bool f0 = bitRead(instr, 4);
  if (curFunc[0] != f0) {
    curFunc[0] = f0;
    changedFunc = true;
  }

  for (int i = 0; i < 4; i++) {
    bool fi = bitRead(instr, i);
    if (curFunc[1 + i] != fi) {
      curFunc[1 + i] = fi;
      changedFunc = true;
    }
  }
}

void decodeFunctions_F5_F8(byte instr, bool& changedFunc) {
  for (int i = 0; i < 4; i++) {
    bool fi = bitRead(instr, i);
    if (curFunc[5 + i] != fi) {
      curFunc[5 + i] = fi;
      changedFunc = true;
    }
  }
}

void decodeFunctions_F9_F12(byte instr, bool& changedFunc) {
  for (int i = 0; i < 4; i++) {
    bool fi = bitRead(instr, i);
    if (curFunc[9 + i] != fi) {
      curFunc[9 + i] = fi;
      changedFunc = true;
    }
  }
}

void decodeFunctions_F13_F20(byte dataByte, bool& changedFunc) {
  for (int i = 0; i < 8; i++) {
    int idx = 13 + i;
    bool fi = bitRead(dataByte, i);
    if (idx < 29 && curFunc[idx] != fi) {
      curFunc[idx] = fi;
      changedFunc = true;
    }
  }
}

void decodeFunctions_F21_F28(byte dataByte, bool& changedFunc) {
  for (int i = 0; i < 8; i++) {
    int idx = 21 + i;
    bool fi = bitRead(dataByte, i);
    if (idx < 29 && curFunc[idx] != fi) {
      curFunc[idx] = fi;
      changedFunc = true;
    }
  }
}

void decodeSpeed14_28(byte instr, bool& changedSpeed) {
  bool forward = bitRead(instr, 5);

  byte v = (instr & 0b00001111) << 1;
  bitWrite(v, 0, bitRead(instr, 4));   // zusätzl. Speed-Bit

  int newSpeed;
  switch (v) {
    case 0:
    case 1:
    case 2:
    case 3:
      newSpeed = 0;   // stop / estop zusammengefasst
      break;
    default:
      newSpeed = (int)v - 3;  // 1..28
      break;
  }

  if (curDir != forward) {
    curDir = forward;
    changedSpeed = true;
  }
  if (curSpeed != newSpeed) {
    curSpeed = newSpeed;
    changedSpeed = true;
  }
}

void decodeSpeed128(byte speedByte, bool& changedSpeed) {
  bool forward = bitRead(speedByte, 7);
  byte v = speedByte & 0x7F;

  int newSpeed;
  if (v == 0 || v == 1) newSpeed = 0;   // stop / estop zusammengefasst
  else newSpeed = (int)v - 1;           // 1..126

  if (curDir != forward) {
    curDir = forward;
    changedSpeed = true;
  }
  if (curSpeed != newSpeed) {
    curSpeed = newSpeed;
    changedSpeed = true;
  }
}

void decodePacket(const byte* local, byte len) {
  if (!packetXorOk(local, len)) return;

  int adr;
  bool isLong;
  byte instrIndex;

  if (!decodeLocoAddress(local, len, adr, isLong, instrIndex)) return;

  if (TARGET_ADDR >= 0 && adr != TARGET_ADDR) return;
  if (instrIndex >= len - 1) return;  // kein Instruktionsbyte

  byte instr = local[instrIndex];

  printRawPacket(adr, isLong, local, len, instrIndex);
  applyAddressContext(adr, isLong);

  bool changedSpeed = false;
  bool changedFunc = false;

  // ----------------------------------------------------------
  // Funktionen F0..F4 : 100xxxxx
  // Beispiele aus deiner Ausgabe: 0x90, 0x91
  // ----------------------------------------------------------
  if ((instr & 0b11100000) == 0b10000000) {
    decodeFunctions_F0_F4(instr, changedFunc);
  }

  // ----------------------------------------------------------
  // Funktionen F5..F8 : 1011xxxx
  // Beispiele: 0xB0, 0xB1, 0xB8
  // ----------------------------------------------------------
  if ((instr & 0b11110000) == 0b10110000) {
    decodeFunctions_F5_F8(instr, changedFunc);
  }

  // ----------------------------------------------------------
  // Funktionen F9..F12 : 1010xxxx
  // ----------------------------------------------------------
  if ((instr & 0b11110000) == 0b10100000) {
    decodeFunctions_F9_F12(instr, changedFunc);
  }

  // ----------------------------------------------------------
  // Fahrstufe 14/28 : 01xxxxxx
  // Beispiele: 0x40, 0x41, 0x55, 0x60, 0x61, 0x75
  // ----------------------------------------------------------
  if ((instr & 0b11000000) == 0b01000000) {
    decodeSpeed14_28(instr, changedSpeed);
  }

  // ----------------------------------------------------------
  // Fahrstufe 128 : 0x3F + Folgebyte
  // Beispiel: 16 3F 80 A9
  // ----------------------------------------------------------
  if (instr == 0x3F && (instrIndex + 1) < (len - 1)) {
    decodeSpeed128(local[instrIndex + 1], changedSpeed);
  }

  // ----------------------------------------------------------
  // Erweiterte Funktionen
  // 0xDE + Byte => F13..F20
  // 0xDF + Byte => F21..F28
  // ----------------------------------------------------------
  if (instr == 0xDE && (instrIndex + 1) < (len - 1)) {
    decodeFunctions_F13_F20(local[instrIndex + 1], changedFunc);
  }

  if (instr == 0xDF && (instrIndex + 1) < (len - 1)) {
    decodeFunctions_F21_F28(local[instrIndex + 1], changedFunc);
  }

  if (changedSpeed && changedFunc) {
    maybePrint("both", true, true);
  } else if (changedSpeed) {
    maybePrint("speed", true, false);
  } else if (changedFunc) {
    maybePrint("functions", false, true);
  }
}

// ============================================================
// Setup / Loop
// ============================================================

void setup() {
  Serial.begin(115200);
  pinMode(dccPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(dccPin), dccISR, CHANGE);
  resetDecoder();
  resetState();

  Serial.println("DCC Lok-Sniffer neu aufgebaut");
  Serial.println("Kommandos: ALL, S<num>, L<num>");
}

void loop() {
  handleSerial();

  if (packetReady) {
    noInterrupts();
    byte len = packetLen;
    byte local[8];
    for (byte i = 0; i < len; i++) local[i] = packet[i];
    packetReady = false;
    interrupts();

    decodePacket(local, len);
  }
}
