#include <Arduino.h>

const byte dccPin = 2;

// ============================================================
// Ziel-Datentyp
// ============================================================

struct lokdada {
  bool richtung;         // true = vorwärts
  int geschwindigkeit;   // 0..127 normiert
  int funktionen[29];    // F0..F28, jeweils 0 oder 1
};

lokdada lokData;

// optional: zum Erkennen von Änderungen
lokdada prevLokData;

// Adressfilter
volatile int TARGET_ADDR = -1;   // -1 = alle, sonst kurze/lange Adresse

// nur für Diagnose
int currentDecodedAddr = -1;
bool currentDecodedIsLong = false;

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
    finishPacket();
  }
}

void dccISR() {
  unsigned long now = micros();
  unsigned long dt = now - lastEdge;
  lastEdge = now;

  int halfType = -1;

  // An deine Messung angepasst
  if (dt >= 44 && dt <= 68) {
    halfType = 1;   // kurze Halbwelle = DCC "1"
  } else if (dt >= 90 && dt <= 700) {
    halfType = 0;   // lange Halbwelle = DCC "0"
  } else {
    pendingHalf = -1;
    return;
  }

  if (pendingHalf == -1) {
    pendingHalf = halfType;
    return;
  }

  if (pendingHalf == halfType) {
    pushBit((byte)halfType);
  }

  pendingHalf = -1;
}

// ============================================================
// Hilfsfunktionen
// ============================================================

void resetLokData(lokdada &d) {
  d.richtung = true;
  d.geschwindigkeit = 0;
  for (int i = 0; i < 29; i++) d.funktionen[i] = 0;
}

void copyLokData(const lokdada &src, lokdada &dst) {
  dst.richtung = src.richtung;
  dst.geschwindigkeit = src.geschwindigkeit;
  for (int i = 0; i < 29; i++) dst.funktionen[i] = src.funktionen[i];
}

bool lokDataEquals(const lokdada &a, const lokdada &b) {
  if (a.richtung != b.richtung) return false;
  if (a.geschwindigkeit != b.geschwindigkeit) return false;
  for (int i = 0; i < 29; i++) {
    if (a.funktionen[i] != b.funktionen[i]) return false;
  }
  return true;
}

// Normierung auf 0..127
int normalizeSpeed(int rawStep, int rawMax) {
  if (rawStep <= 0) return 0;
  if (rawMax <= 0) return 0;

  long num = (long)(rawStep - 1) * 126L;
  long den = (long)(rawMax - 1);
  long scaled = 1L + (num + den / 2) / den;   // gerundet

  if (scaled < 1) scaled = 1;
  if (scaled > 127) scaled = 127;

  return (int)scaled;
}

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

void printLokData(const lokdada &d) {
  Serial.print("richtung: ");
  Serial.println(d.richtung ? "true" : "false");

  Serial.print("geschwindigkeit: ");
  Serial.println(d.geschwindigkeit);

  Serial.print("funktionen: [");
  for (int i = 0; i < 29; i++) {
    if (i) Serial.print(", ");
    Serial.print(d.funktionen[i]);
  }
  Serial.println("]");
}

// ============================================================
// Decoder in lokdada
// ============================================================

void decodeFunctions_F0_F4(byte instr, lokdada &d) {
  d.funktionen[0] = bitRead(instr, 4) ? 1 : 0;   // F0

  for (int i = 0; i < 4; i++) {
    d.funktionen[1 + i] = bitRead(instr, i) ? 1 : 0;   // F1..F4
  }
}

void decodeFunctions_F5_F8(byte instr, lokdada &d) {
  for (int i = 0; i < 4; i++) {
    d.funktionen[5 + i] = bitRead(instr, i) ? 1 : 0;
  }
}

void decodeFunctions_F9_F12(byte instr, lokdada &d) {
  for (int i = 0; i < 4; i++) {
    d.funktionen[9 + i] = bitRead(instr, i) ? 1 : 0;
  }
}

void decodeFunctions_F13_F20(byte value, lokdada &d) {
  for (int i = 0; i < 8; i++) {
    d.funktionen[13 + i] = bitRead(value, i) ? 1 : 0;
  }
}

void decodeFunctions_F21_F28(byte value, lokdada &d) {
  for (int i = 0; i < 8; i++) {
    d.funktionen[21 + i] = bitRead(value, i) ? 1 : 0;
  }
}

void decodeSpeed14_28(byte instr, lokdada &d) {
  bool forward = bitRead(instr, 5);

  byte v = (instr & 0b00001111) << 1;
  bitWrite(v, 0, bitRead(instr, 4));   // zusätzliches Speed-Bit

  int step28;
  switch (v) {
    case 0:
    case 1:
    case 2:
    case 3:
      step28 = 0;  // Stop / E-Stop zusammengefasst
      break;
    default:
      step28 = (int)v - 3;   // 1..28
      break;
  }

  d.richtung = forward;
  d.geschwindigkeit = normalizeSpeed(step28, 28);
}

void decodeSpeed128(byte speedByte, lokdada &d) {
  bool forward = bitRead(speedByte, 7);
  byte v = speedByte & 0x7F;

  int step126;
  if (v == 0 || v == 1) step126 = 0;   // Stop / E-Stop zusammengefasst
  else step126 = (int)v - 1;           // 1..126

  d.richtung = forward;
  d.geschwindigkeit = normalizeSpeed(step126, 126);
}

void decodePacketToLokData(const byte* local, byte len, lokdada &d) {
  if (!packetXorOk(local, len)) return;

  int adr;
  bool isLong;
  byte instrIndex;

  if (!decodeLocoAddress(local, len, adr, isLong, instrIndex)) return;
  if (TARGET_ADDR >= 0 && adr != TARGET_ADDR) return;
  if (instrIndex >= len - 1) return;

  currentDecodedAddr = adr;
  currentDecodedIsLong = isLong;

  byte instr = local[instrIndex];

  // F0..F4 : 100xxxxx
  if ((instr & 0b11100000) == 0b10000000) {
    decodeFunctions_F0_F4(instr, d);
  }

  // F5..F8 : 1011xxxx
  if ((instr & 0b11110000) == 0b10110000) {
    decodeFunctions_F5_F8(instr, d);
  }

  // F9..F12 : 1010xxxx
  if ((instr & 0b11110000) == 0b10100000) {
    decodeFunctions_F9_F12(instr, d);
  }

  // 14/28 Fahrstufen : 01xxxxxx
  if ((instr & 0b11000000) == 0b01000000) {
    decodeSpeed14_28(instr, d);
  }

  // 128 Fahrstufen : 0x3F + Folgebyte
  if (instr == 0x3F && (instrIndex + 1) < (len - 1)) {
    decodeSpeed128(local[instrIndex + 1], d);
  }

  // Erweiterte Funktionen
  if (instr == 0xDE && (instrIndex + 1) < (len - 1)) {
    decodeFunctions_F13_F20(local[instrIndex + 1], d);
  }

  if (instr == 0xDF && (instrIndex + 1) < (len - 1)) {
    decodeFunctions_F21_F28(local[instrIndex + 1], d);
  }
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
    resetLokData(lokData);
    resetLokData(prevLokData);
    currentDecodedAddr = -1;
    currentDecodedIsLong = false;
    Serial.println("Filter: ALL");
    return;
  }

  if (s.charAt(0) == 'S' || s.charAt(0) == 's') {
    int adr = s.substring(1).toInt();
    if (adr >= 1 && adr <= 127) {
      TARGET_ADDR = adr;
      resetLokData(lokData);
      resetLokData(prevLokData);
      currentDecodedAddr = -1;
      currentDecodedIsLong = false;
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
      resetLokData(lokData);
      resetLokData(prevLokData);
      currentDecodedAddr = -1;
      currentDecodedIsLong = false;
      Serial.print("Filter lange Adresse: ");
      Serial.println(TARGET_ADDR);
    } else {
      Serial.println("Langadresse muss 128..10239 sein");
    }
    return;
  }

  if (s.equalsIgnoreCase("PRINT")) {
    Serial.print("adresse: ");
    Serial.println(currentDecodedAddr);
    Serial.print("adressmodus: ");
    Serial.println(currentDecodedIsLong ? "long" : "short");
    printLokData(lokData);
    return;
  }

  Serial.println("Kommandos: ALL, S<num>, L<num>, PRINT");
}

// ============================================================
// Setup / Loop
// ============================================================

void setup() {
  Serial.begin(115200);
  pinMode(dccPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(dccPin), dccISR, CHANGE);

  resetDecoder();
  resetLokData(lokData);
  resetLokData(prevLokData);

  Serial.println("DCC Decoder mit lokdada gestartet");
  Serial.println("Kommandos: ALL, S<num>, L<num>, PRINT");
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

    decodePacketToLokData(local, len, lokData);

    // Nur für Diagnose: bei Änderung ausgeben
    if (!lokDataEquals(lokData, prevLokData)) {
      Serial.print("adresse: ");
      Serial.println(currentDecodedAddr);
      Serial.print("adressmodus: ");
      Serial.println(currentDecodedIsLong ? "long" : "short");
      printLokData(lokData);
      Serial.println();

      copyLokData(lokData, prevLokData);
    }
  }
}
