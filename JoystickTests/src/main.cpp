#include <SerialTransfer.h>

SerialTransfer transfer;

float target_position = 0.0;
float received_position = 0.0;

void setup() {
  Serial.begin(9600);            // For user input/debugging
  Serial.println("serial setup");
}

void loop() {
  // Example input from user: T3.14
  float joystickX = (analogRead(39) * 100 / 4095.0) - 50; // Read joystick X-axis
  if (abs(joystickX) > 5){
    Serial.print("Joystick X: ");
    Serial.println(joystickX);
  }
  delay(100);
}
