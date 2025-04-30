#include <Arduino.h>
#define POT A0
#define LED_BUILTIN 13


// put function declarations here:
void flashLED(int led, int delayTime);
double readPOT(int potPin);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(POT, INPUT);
  Serial.println("Setup complete");
  
}

void loop() {
  // put your main code here, to run repeatedly:
  double potValue = readPOT(POT);
  flashLED(LED_BUILTIN, potValue);

  Serial.println(potValue);
}

// put function definitions here:
void flashLED(int led, int delayTime) {
  unsigned static long lastFlashTime = 0;
  if (millis() - lastFlashTime > delayTime) {
    digitalWrite(led, !digitalRead(led));
    lastFlashTime = millis();
  }
  delay(3);
}
double readPOT(int pot) {
  return analogRead(pot);
}