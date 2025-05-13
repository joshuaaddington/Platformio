#include <SimpleFOC.h>
#include <SerialTransfer.h>
#include <esp_now.h>
#include <WiFi.h>

// Struct for receiving and replying
typedef struct struct_message {
  float target_velocity;
  float current_velocity;
} struct_message;

struct_message incomingData;
struct_message outgoingData;

// MAC address of Main Board
// d0:ef:76:5c:fd:90
 uint8_t receiverMac[] = {0xD0, 0xEF, 0x76, 0x5C, 0xFD, 0x90};

float current_velocity = 0.0;
float target_velocity = 0.0;

// I2C sensor and bus
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);

// Motor and driver
BLDCMotor motor = BLDCMotor(7); // 7 pole pairs
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

// Callback function for ESP-NOW
// This function is called when data is received
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));
  Serial.print("Target velocity received: ");
  Serial.println(incomingData.target_velocity);

  // Update motor here
  target_velocity = incomingData.target_velocity;
  outgoingData.current_velocity = current_velocity;

  // Send response
  esp_now_send(receiverMac, (uint8_t *) &outgoingData, sizeof(outgoingData));
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

  Serial.println("ESP-NOW setup done.");

  // I2C sensor setup (SDA=19, SCL=18)
  I2Cone.begin(19, 18, 400000);
  sensor.init(&I2Cone);
  motor.linkSensor(&sensor);

  // Motor driver setup
  driver.voltage_power_supply = 24;
  driver.init();
  motor.linkDriver(&driver);

  // FOC config
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::velocity;

  // PID tuning
  motor.PID_velocity.P = .1;
  motor.PID_velocity.I = 1.0;
  motor.PID_velocity.D = 0.0;


  // Limits
  motor.voltage_limit = 24;
  motor.velocity_limit = 1000;
  motor.LPF_velocity.Tf = 0.01;

  motor.init();
  motor.initFOC();

  Serial.println("Motor ready.");
}

void loop() {
  motor.loopFOC();

  // Move to new position
  motor.move(target_velocity);

  // // Send current position back
  // float current_velocity = motor.shaft_velocity;
  // Serial.print("Current velocity: ");
  // Serial.println(current_velocity);
  // esp_now_send(receiverMac, (uint8_t *) &current_velocity, sizeof(current_velocity));
}
