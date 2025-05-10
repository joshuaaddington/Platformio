#include <SerialTransfer.h>

SerialTransfer transfer;

float target_position = 0.0;
float received_position = 0.0;

void setup() {
  Serial.begin(115200);            // For user input/debugging
  Serial1.begin(115200, SERIAL_8N1, 16, 17); // UART to motor board (TX=17, RX=16)
  transfer.begin(Serial1);

  Serial.println("Main controller ready.");
}

void loop() {
  // Example input from user: T3.14
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    if (input.charAt(0) == 'T') {
      target_position = input.substring(1).toFloat();
    }
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

  delay(50); // Adjust as needed for responsiveness
}
