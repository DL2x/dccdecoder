// == DCC LOK-SNIFFER (eine Adresse, umschaltbar) ==
// - Serial Kommandos: S<num> (kurz), L<num> (lang), ALL (Filter aus)
// - Ausgabe (immer gleiche Felder):
//   typ: speed|functions
//   fahrstufe: <int>   (0 = STOP, >0 = Fahrstufe)
//   richtung: true|false (true = vorwärts)
//   funktionen: [29 Booleans F0..F28]

#include <TimerOne.h>

#define DCC_SHORT_ADDRESS  0x00
#define DCC_LONG_ADDRESS   0x01
#define DCC_ACC_ADDRESS    0x02


// -------- Filter: genau EINE Variable (wird zur Laufzeit geändert) --------
volatile int TARGET_ADDR = 4711;  // -1 = alle Lokpakete, sonst 1..127 kurz, >=128 lang

// Eingänge:
const int dccPin = 2;      // UNO INT0
const int ledPin = 13;

// Variablen für DCC-Erkennung (ISR <-> loop)
const byte MAX_DATA = 8;
volatile byte dataBuf[MAX_DATA];   // Frame-Puffer (ISR schreibt)
volatile byte datalength = 0;      // Index im dataBuf
volatile byte countbit = 0;        // Bits pro Byte zählen
volatile byte dxor = 0;            // XOR-Prüfsumme
volatile int  countone = 0;        // Präambel-1-Bits
volatile boolean getdata = false;  // wir lesen gerade
volatile boolean dataReady = false;

// --------- Zustände für unsere Ziel-Lok ----------
int curSpeed = -1;            // -1 unbekannt, 0 STOP, sonst >0
bool curDir   = true;         // true = vorwärts
bool curFunc[29];             // F0..F28

// Vorherige Zustände (für Dedupe)
int  prevSpeed = -1;
bool prevDir   = true;
bool prevFunc[29];

// ------------------ Hilfen ----------------------
void printState(const char* typ) {
  Serial.print("typ: ");
  Serial.println(typ);

  Serial.print("fahrstufe: ");
  Serial.println(curSpeed);

  Serial.print("richtung: ");
  Serial.println(curDir ? "true" : "false");

  Serial.print("funktionen: [");
  for (int i = 0; i < 29; i++) {
    if (i) Serial.print(", ");
    Serial.print(curFunc[i] ? "true" : "false");
  }
  Serial.println("]");
}

// Nur ausgeben, wenn sich etwas geändert hat
void maybePrint(const char* typ, bool changedSpeed, bool changedFunc) {
  if (!changedSpeed && !changedFunc) return;
  printState(typ);

  // Snapshot für Dedupe
  prevSpeed = curSpeed;
  prevDir   = curDir;
  for (int i=0;i<29;i++) prevFunc[i] = curFunc[i];
}

inline void resetState() {
  curSpeed = -1; prevSpeed = -1;
  curDir = true; prevDir = true;
  for (int i=0;i<29;i++) { curFunc[i]=false; prevFunc[i]=false; }
}

// ---------------- Serial-Kommandos ----------------
void handleSerialCmd() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n'); s.trim();
  if (s.length()==0) return;

  if (s.equalsIgnoreCase("ALL")) {
    TARGET_ADDR = -1;
    resetState();
    Serial.println("Filter aus (alle Lokpakete)");
    return;
  }
  if (s.charAt(0) == 'S' || s.charAt(0) == 's') {
    int v = s.substring(1).toInt();
    if (v >= 1 && v <= 127) {
      TARGET_ADDR = v;
      resetState();
      Serial.print("Filter kurze Adresse -> "); Serial.println(TARGET_ADDR);
    } else {
      Serial.println("Ungueltige Kurzadresse (1..127).");
    }
    return;
  }
  if (s.charAt(0) == 'L' || s.charAt(0) == 'l') {
    int v = s.substring(1).toInt();
    if (v >= 128 && v <= 16383) {
      TARGET_ADDR = v;
      resetState();
      Serial.print("Filter lange Adresse -> "); Serial.println(TARGET_ADDR);
    } else {
      Serial.println("Ungueltige Langadresse (128..16383).");
    }
    return;
  }

  Serial.println("Kommandos: S<num>, L<num>, ALL");
}

// *********************************************************
// Interrupt-Funktion auf dccPin
void dccdata() {
  Timer1.start();  // Timer starten (Einmal-Delay)
}

// *********************************************************
// Timer nach Interrupt auf DCC-Pin (ca. 70 µs später)
void dcctime() {
  Timer1.stop();  // Timer anhalten
  int State = digitalRead(dccPin);

  if (getdata == true) {
    countbit += 1;  // Bits im aktuellen Byte mitzählen
  }

  if (State == LOW) {  // ------- 1-Bit -------
    countone += 1;
    if (getdata == true && countbit <= 8 && datalength < MAX_DATA) {
      bitWrite(dataBuf[datalength], 8 - countbit, 1);
    }
    if (countbit > 8) {  // End-Bit nach letztem Datenbyte
      countbit = 0;
      getdata = false;

      // XOR prüfen: letztes Byte muss gleich dxor sein
      if (dataBuf[datalength] != dxor) {
        // verwerfen
        datalength = 0; dxor = 0;
        return;
      }

      // restliche Bytes leeren (sauber, ohne Überlauf)
      for (byte i = datalength + 1; i < MAX_DATA; i++) dataBuf[i] = 0;

      dataReady = true;  // Fertig!
    }
  } else {               // ------- 0-Bit (Separator/long) -------
    if (getdata == true && countbit <= 8 && datalength < MAX_DATA) {
      bitWrite(dataBuf[datalength], 8 - countbit, 0);
    }
    if (countone > 10) {   // Präambel erkannt (>=10 Einsen, dann 0)
      getdata = true;
      datalength = 0;
      countbit = 0;
      dxor = 0;
      for (byte i=0;i<MAX_DATA;i++) dataBuf[i]=0;
    }
    if (countbit > 8) {    // Byte fertig -> Separator
      countbit = 0;
      dxor = dxor ^ dataBuf[datalength];  // XOR fortschreiben
      if (datalength + 1 < MAX_DATA) {
        datalength += 1;                  // nächstes Byte
      } else {
        // Buffer voll -> Aufnahme stoppen
        getdata = false;
      }
    }
    countone = 0;    // mit 0-Bit endet Präambel-Zählung
  }
}

// *********************************************************
// Auswerten des eingelesenen Frames (nur auf lokalem Snapshot!)
void dccauswertung(const byte* dataL, byte len) {
  // Idle? (einfach: 0xFF, 0x00, ..., 0xFF)
  if (len >= 3 && dataL[0]==0xFF && dataL[1]==0x00 && dataL[len-1]==0xFF) return;

  // --- Adresse bestimmen ---
  int dccAdr = 0;
  byte dccType = 0xFF;
  if (bitRead(dataL[0],7) == 0) {                 // kurze Adresse
    dccAdr = dataL[0];
    dccType = DCC_SHORT_ADDRESS;
    if (dccAdr == 0xFF || dccAdr == 0) return;    // idle
    if (TARGET_ADDR >= 0) {                       // Filter
      if ((TARGET_ADDR <= 127 && TARGET_ADDR != dccAdr) || (TARGET_ADDR >= 128)) return;
    }
  }
  else if ( (dataL[0] >> 6) == B11 ) {            // lange Adresse
    dccAdr = word(dataL[0] & B00111111, dataL[1]);
    dccType = DCC_LONG_ADDRESS;
    if (dccAdr == 16128) return;                  // idle
    if (TARGET_ADDR >= 0) {
      if ((TARGET_ADDR >= 128 && TARGET_ADDR != dccAdr) || (TARGET_ADDR <= 127)) return;
    }
  }
  else if ( ((dataL[0] >> 6) == B10) && ((dataL[1] >> 7) == 1) ) {
    // Zubehör -> ignorieren
    return;
  } else {
    return; // unbekannt
  }

  byte verschub = (dccType == DCC_LONG_ADDRESS) ? 1 : 0;
  if (len <= 1+verschub) return;
  byte instr = dataL[1+verschub];

  bool changedSpeed = false;
  bool changedFunc  = false;

  // ---- Funktionen F0..F4 ----
  if ( ((instr >> 5) & B111) == B100 ) {
    bool f0 = bitRead(instr,4);
    if (curFunc[0] != f0) { curFunc[0] = f0; changedFunc = true; }
    for (int i=0;i<4;i++) {
      bool fi = bitRead(instr,i);
      if (curFunc[1+i] != fi) { curFunc[1+i] = fi; changedFunc = true; }
    }
  }

  // ---- Funktionen F5..F8 ----
  if ( ((instr >> 4) & B1111) == B1011 ) {
    for (int i=0;i<4;i++) {
      bool fi = bitRead(instr,i);
      if (curFunc[5+i] != fi) { curFunc[5+i] = fi; changedFunc = true; }
    }
  }

  // ---- Funktionen F9..F12 ----
  if ( ((instr >> 4) & B1111) == B1010 ) {
    for (int i=0;i<4;i++) {
      bool fi = bitRead(instr,i);
      if (curFunc[9+i] != fi) { curFunc[9+i] = fi; changedFunc = true; }
    }
  }

  // ---- Fahrstufe 14/28 ----
  if ( (instr >> 6) == B01 ) {
    bool forward = bitRead(instr,5);
    byte v = (instr & B1111) << 1;
    bitWrite(v,0,bitRead(instr,4));  // additional speed bit

    int newSpeed;
    switch (v) {
      case 0: newSpeed = 0; break;            // STOP
      case 1: newSpeed = 0; break;            // STOP (I)
      case 2: newSpeed = 0; break;            // ESTOP
      case 3: newSpeed = 0; break;            // ESTOP (I)
      default: newSpeed = (int)v - 3;         // 1..28
    }

    if (curDir != forward) { curDir = forward; changedSpeed = true; }
    if (curSpeed != newSpeed) { curSpeed = newSpeed; changedSpeed = true; }
  }

  // ---- Fahrstufe 128 (0x3F + Byte) ----
  if ( instr == B00111111 && (len > 2+verschub) ) {
    byte b = dataL[2+verschub];
    bool forward = bitRead(b,7);
    int newSpeed;
    byte v = b & B01111111;
    if      (v == 0) newSpeed = 0;    // STOP
    else if (v == 1) newSpeed = 0;    // ESTOP
    else             newSpeed = (int)v - 1; // 1..126

    if (curDir != forward) { curDir = forward; changedSpeed = true; }
    if (curSpeed != newSpeed) { curSpeed = newSpeed; changedSpeed = true; }
  }

  // ---- F13..F20 (0xDE + Byte) ----
  if ( instr == B11011110 && (len > 2+verschub) ) {
    byte b = dataL[2+verschub];
    for (int i=0;i<8;i++) {
      bool fi = bitRead(b,i);
      int idx = 13+i; // F13..F20
      if (idx < 29 && curFunc[idx] != fi) { curFunc[idx] = fi; changedFunc = true; }
    }
  }

  // ---- F21..F28 (0xDF + Byte) ----
  if ( instr == B11011111 && (len > 2+verschub) ) {
    byte b = dataL[2+verschub];
    for (int i=0;i<8;i++) {
      bool fi = bitRead(b,i);
      int idx = 21+i; // F21..F28
      if (idx < 29 && curFunc[idx] != fi) { curFunc[idx] = fi; changedFunc = true; }
    }
  }

  // Ausgabe bei Änderung(en)
  if (changedSpeed && changedFunc)      maybePrint("both", true, true);
  else if (changedSpeed)                maybePrint("speed", true, false);
  else if (changedFunc)                 maybePrint("functions", false, true);
}

// =================== Setup / Loop =====================
void setup() {
  // I/O
  pinMode(dccPin, INPUT);   // Pullup extern
  pinMode(ledPin, OUTPUT);

  // Interrupts/Timer
  attachInterrupt(digitalPinToInterrupt(dccPin), dccdata, RISING);  // INT0 (Pin 2)
  Timer1.initialize(70);                // ~70 µs
  Timer1.attachInterrupt(dcctime);
  Timer1.stop();

  // Serial
  Serial.begin(115200);
  Serial.println("DCC-Sniffer gestartet (umschaltbare Adresse)");
  Serial.println("Kommandos: S<num> (kurz), L<num> (lang), ALL");

  // Startzustand
  resetState();
}

void loop() {
  digitalWrite(ledPin, !getdata);

  handleSerialCmd();

  if (dataReady == true) {
    // lokales Snapshot (kein Race)
    noInterrupts();
    byte len = datalength + 1; if (len > MAX_DATA) len = MAX_DATA;
    byte local[MAX_DATA];
    for (byte i=0;i<len;i++) local[i] = dataBuf[i];
    dataReady = false;
    interrupts();

    dccauswertung(local, len);
  }
}
