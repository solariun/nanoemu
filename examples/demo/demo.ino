// nanoemu demo: blink, PWM fade on every PWM pin, ADC readout,
// INT0 button counter, EEPROM boot counter and a serial echo.
#include <EEPROM.h>

volatile unsigned long presses = 0;
void onButton() { presses++; }

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), onButton, FALLING);
  byte boots = EEPROM.read(0) + 1;
  EEPROM.write(0, boots);
  Serial.print(F("nanoemu demo - boot #"));
  Serial.println(boots);
  Serial.println(F("Type something: it is echoed back in UPPERCASE."));
}

void loop() {
  static unsigned long last = 0;
  static uint8_t fade = 0;
  static int8_t dir = 5;
  unsigned long now = millis();
  if (now - last >= 500) {
    last = now;
    digitalWrite(13, !digitalRead(13));
    Serial.print(F("t="));
    Serial.print(now);
    Serial.print(F("ms A0="));
    Serial.print(analogRead(A0));
    Serial.print(F(" A1="));
    Serial.print(analogRead(A1));
    Serial.print(F(" presses="));
    Serial.println(presses);
  }
  analogWrite(3, fade);
  analogWrite(5, 255 - fade);
  analogWrite(6, 64);
  analogWrite(9, 128);
  analogWrite(10, 192);
  analogWrite(11, fade / 2 + 1);
  fade += dir;
  if (fade == 0 || fade == 255) dir = -dir;
  delay(10);
  while (Serial.available()) Serial.write(toupper(Serial.read()));
}
