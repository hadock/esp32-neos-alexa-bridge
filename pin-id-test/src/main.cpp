// Pinout identity check. GPIO19 (red LED) blinks TWICE on its turn, then
// GPIO20 (green LED) blinks ONCE on its turn, with a pause between turns.
// The double-blink vs single-blink makes it unmistakable which LED is
// which, independent of color perception or timing sync.

#include <Arduino.h>

static const int PIN_19 = 19;
static const int PIN_20 = 20;

static void blinkTimes(int pin, int times, int onMs = 250, int offMs = 250) {
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(onMs);
    digitalWrite(pin, LOW);
    delay(offMs);
  }
}

void setup() {
  pinMode(PIN_19, OUTPUT);
  pinMode(PIN_20, OUTPUT);
  digitalWrite(PIN_19, LOW);
  digitalWrite(PIN_20, LOW);
}

void loop() {
  blinkTimes(PIN_19, 2);  // red: two blinks
  delay(700);
  blinkTimes(PIN_20, 1);  // green: one blink
  delay(700);
}
