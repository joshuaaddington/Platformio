#include <esp_now.h>
#include <WiFi.h>

typedef struct struct_message {
  float target_position;
  float current_position; // This is what will be received
} struct_message;

struct_message outgoingData;
struct_message incomingData;

// Maximum velocity for the motor
float max_velocity = 100.0;

// MAC address of the receiver ESP32 (Motor Controller)
// 30:c6:f7:31:a1:f4
uint8_t receiverMac[] = {0x30, 0xc6, 0xf7, 0x31, 0xa1, 0xf4};

void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));
  Serial.print("Current position: ");
  Serial.println(incomingData.current_position);
}


void setup() {
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);
  Serial.println("Sender MAC: " + WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("ESP-NOW sender ready.");
}

unsigned long lastCommandTime = 0;
bool commandRecentlySent = false;

void loop() {
  // Simulate joystick input
  float input = analogRead(39) * (max_velocity * 2) / 4095.0 - (max_velocity);

  // Check if the input is outside the deadzone
  if (abs(input) > (max_velocity / 7.5)) {
    outgoingData.target_position = input;
    Serial.print("Sending target velocity: ");
    Serial.println(outgoingData.target_position);

    esp_now_send(receiverMac, (uint8_t *) &outgoingData, sizeof(outgoingData));
    lastCommandTime = millis();
    commandRecentlySent = true;
  } else {
    // If no valid command recently and timeout exceeded, send 0
    if (commandRecentlySent && millis() - lastCommandTime > 50) {
      outgoingData.target_position = 0.0;
      Serial.println("Sending timeout command: 0.0");

      esp_now_send(receiverMac, (uint8_t *) &outgoingData, sizeof(outgoingData));
      commandRecentlySent = false;  // Avoid repeated 0 commands
    }
  }

  delay(20);
}

