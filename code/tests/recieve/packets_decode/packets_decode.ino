const byte dccPin = 2;

// -1 = alle Lokpakete
volatile int TARGET_ADDR = -1;

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

  // Schwellen anhand deiner Messung
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

void handleSerial() {
  if (!Serial.available()) return;

  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() == 0) return;

  if (s.equalsIgnoreCase("ALL")) {
    TARGET_ADDR = -1;
    Serial.println("Filter: ALL");
    return;
  }

  if (s.charAt(0) == 'S' || s.charAt(0) == 's') {
    int adr = s.substring(1).toInt();
    if (adr >= 1 && adr <= 127) {
      TARGET_ADDR = adr;
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
      Serial.print("Filter lange Adresse: ");
      Serial.println(TARGET_ADDR);
    } else {
      Serial.println("Langadresse muss 128..10239 sein");
    }
    return;
  }

  Serial.println("Kommandos: ALL, S<num>, L<num>");
}

// liefert true, wenn Lokadresse erkannt wurde
bool decodeLocoAddress(const byte* local, byte len, int& adr, bool& isLong, byte& instrIndex) {
  adr = -1;
  isLong = false;
  instrIndex = 1;

  if (len < 3) return false;

  // Idle ignorieren
  if (len >= 3 && local[0] == 0xFF && local[1] == 0x00 && local[len - 1] == 0xFF) {
    return false;
  }

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

void setup() {
  Serial.begin(115200);
  pinMode(dccPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(dccPin), dccISR, CHANGE);
  resetDecoder();
  Serial.println("DCC Packet Test gestartet");
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

    if (len >= 3) {
      byte x = 0;
      for (byte i = 0; i < len - 1; i++) x ^= local[i];

      if (x != local[len - 1]) {
        return;
      }

      int adr;
      bool isLong;
      byte instrIndex;

      if (!decodeLocoAddress(local, len, adr, isLong, instrIndex)) {
        return;
      }

      if (TARGET_ADDR >= 0 && adr != TARGET_ADDR) {
        return;
      }

      Serial.print("Adr: ");
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

      Serial.print(" | XOR OK");
      Serial.println();
    }
  }
}
