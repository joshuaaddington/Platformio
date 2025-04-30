#include <Arduino.h>

// put function declarations here:


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(26, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(analogRead(26));
}

// put function definitions here:
