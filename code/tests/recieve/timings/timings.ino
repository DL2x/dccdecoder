volatile unsigned long lastEdgeMicros = 0;
volatile unsigned int pulseWidth = 0;
volatile bool newPulse = false;

const byte dccPin = 2;

void isrDcc() {
  unsigned long now = micros();
  pulseWidth = (unsigned int)(now - lastEdgeMicros);
  lastEdgeMicros = now;
  newPulse = true;
}

void setup() {
  Serial.begin(115200);
  pinMode(dccPin, INPUT);   // oder INPUT_PULLUP testen, falls kein sauberer Pullup existiert
  attachInterrupt(digitalPinToInterrupt(dccPin), isrDcc, CHANGE);
  Serial.println("DCC Flankenmonitor gestartet");
}

void loop() {
  if (newPulse) {
    noInterrupts();
    unsigned int w = pulseWidth;
    newPulse = false;
    interrupts();

    Serial.println(w);
  }
}
