const byte dccPin = 2;

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

void setup() {
  Serial.begin(115200);
  pinMode(dccPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(dccPin), dccISR, CHANGE);
  resetDecoder();
  Serial.println("DCC Packet Test gestartet");
}

void loop() {
  if (packetReady) {
    noInterrupts();
    byte len = packetLen;
    byte local[8];
    for (byte i = 0; i < len; i++) local[i] = packet[i];
    packetReady = false;
    interrupts();

    Serial.print("Paket: ");
    for (byte i = 0; i < len; i++) {
      if (local[i] < 16) Serial.print('0');
      Serial.print(local[i], HEX);
      Serial.print(' ');
    }

    if (len >= 3) {
      byte x = 0;
      for (byte i = 0; i < len - 1; i++) x ^= local[i];

      Serial.print(" | XOR rx=");
      if (local[len - 1] < 16) Serial.print('0');
      Serial.print(local[len - 1], HEX);

      Serial.print(" calc=");
      if (x < 16) Serial.print('0');
      Serial.print(x, HEX);

      Serial.print(x == local[len - 1] ? " OK" : " FAIL");
    }

    Serial.println();
  }
}
