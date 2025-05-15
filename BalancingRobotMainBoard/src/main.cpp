#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define SDA_MPU 21
#define SCL_MPU 22

Adafruit_MPU6050 mpu;

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
  delay(1000);
  Serial.println("Serial started");

  // Initialize ESP-NOW
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

  // Initialize MPU6050
  Wire.begin(SDA_MPU, SCL_MPU);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 found!");

}

unsigned long lastCommandTime = 0;
bool commandRecentlySent = false;

void loop() {
  // Read and print the accelerometer data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  // Compute pitch and roll in radians
  float pitch = atan2(ay, sqrt(ax * ax + az * az));
  float roll  = atan2(-ax, az);

  // Convert to degrees
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;

  // Serial.println("==== Orientation (from Accel) ====");
  // Serial.print("Pitch: "); Serial.print(pitch); Serial.println("°");
  // Serial.print("Roll : "); Serial.print(roll);  Serial.println("°");
  // Serial.println();
  
  // Joystick input
  float input = analogRead(39) * (max_velocity * 2) / 4095.0 - (max_velocity);

  input = pitch;

  // Check if the input is outside the deadzone
  if (abs(input) > (/*(max_velocity / 7.5*/ 0)) {
    outgoingData.target_position = input;

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

  delay(10);
}

