#include <SerialTransfer.h>

SerialTransfer transfer;

float target_position = 0.0;
float received_position = 0.0;

void setup() {
  Serial.begin(9600);            // For user input/debugging
  Serial1.begin(115200, SERIAL_8N1, 17, 16); // UART to motor board (TX=17, RX=16)
  transfer.begin(Serial1);

  Serial.println("Main controller ready.");
}

void loop() {
  // Example input from user: T3.14
  float joystickX = (analogRead(39) * 100 / 4095.0) - 50; // Read joystick X-axis
  if (abs(joystickX) > 5){
    target_position = joystickX;
  }

  // Send position command
  transfer.txObj(target_position);
  transfer.sendData(sizeof(target_position));

  // Receive current position
  if (transfer.available()) {
    transfer.rxObj(received_position);
    Serial.print("Motor position: ");
    Serial.println(received_position, 4);
  }

  delay(20); // Adjust as needed for responsiveness
}
