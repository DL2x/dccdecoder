int pin = 2;

void setup() {
  Serial.begin(115200);
  pinMode(pin, INPUT);
}

void loop() {
  Serial.println(digitalRead(pin));
  delay(1);
}
