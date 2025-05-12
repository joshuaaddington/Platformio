#include <SimpleFOC.h>
#include <SerialTransfer.h>

// I2C sensor and bus
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);

// Motor and driver
BLDCMotor motor = BLDCMotor(7); // Replace with your motor's pole pair count
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

// SerialTransfer object
SerialTransfer transfer;

float target_position = 0.0;

void setup() {
  Serial.begin(115200);   // UART for SerialTransfer
  transfer.begin(Serial);

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
  motor.controller = MotionControlType::angle;

  // PID tuning
  motor.PID_velocity.P = .1;
  motor.PID_velocity.I = 1.0;
  motor.PID_velocity.D = 0.0;

  motor.P_angle.P = 20.0;

  // Limits
  motor.voltage_limit = 6;
  motor.velocity_limit = 100;
  motor.LPF_velocity.Tf = 0.01;

  motor.init();
  motor.initFOC();

  Serial.println("Motor ready.");
}

void loop() {
  motor.loopFOC();

  // Receive new target
  if (transfer.available()) {
    transfer.rxObj(target_position);
  }

  // Move to new position
  motor.move(target_position);

  // Send current position back
  float current_position = motor.shaft_angle;
  transfer.txObj(current_position);
  transfer.sendData(sizeof(current_position));
}