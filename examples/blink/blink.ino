// Classic blink on the on-board LED (D13) plus a heartbeat on serial.
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("on");
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("off");
  delay(250);
}
