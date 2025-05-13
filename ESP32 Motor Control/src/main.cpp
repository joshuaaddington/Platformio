#include <esp_now.h>
#include <WiFi.h>

// Struct for receiving and replying
typedef struct struct_message {
  float target_position;
  float current_position;
} struct_message;

struct_message incomingData;
struct_message outgoingData;

float current_position = 0.0;


void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));
  Serial.print("Target position received: ");
  Serial.println(incomingData.target_position);

  // Update motor here
  current_position += 0.05 * (incomingData.target_position - current_position); // dummy motor move
  outgoingData.current_position = current_position;

  // Send response
  esp_now_send(esp_now_info->src_addr, (uint8_t *) &outgoingData, sizeof(outgoingData));
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.println("Receiver MAC: " + WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("ESP-NOW receiver ready.");
}

void loop() {
  // Your SimpleFOC motor.loopFOC() and motor.move() code would go here
  delay(10);
}
