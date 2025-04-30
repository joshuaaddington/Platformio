/*
MKS DUAL FOC closed-loop position control routine.
Library: SimpleFOC 2.1.1
Hardware: MKS DUAL FOC V3.1

Command: T+position (in radians)
Example: T3.14 => Move to 180 degrees

Make sure to:
- Set your motor's actual pole pair number
- Adjust voltage limits as needed
- Tune PID values for your motor
*/

#include <SimpleFOC.h>

// I2C sensor and bus
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);

// Motor and driver
BLDCMotor motor = BLDCMotor(7); // Update 7 to your motor's pole pair count
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

// Target position in radians
float target_position = 0;

// Serial command interface
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_position, cmd); }

void setup() {
  // Start I2C (SDA=19, SCL=18)
  I2Cone.begin(19, 18, 400000);
  sensor.init(&I2Cone);
  motor.linkSensor(&sensor);

  // Motor driver setup
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // FOC setup
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::angle;

  // PID tuning
  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 1;
  motor.PID_velocity.D = 0.0;

  motor.P_angle.P = 20;

  // Limits
  motor.voltage_limit = 6;   // volts
  motor.velocity_limit = 100; // rad/s max velocity

  motor.LPF_velocity.Tf = 0.01;

  // Initialize serial and motor
  Serial.begin(115200);
  motor.useMonitoring(Serial);

  motor.init();
  motor.initFOC();

  command.add('T', doTarget, "target position");

  Serial.println(F("Motor ready."));
  Serial.println(F("Send target position using: T3.14"));
}

void loop() {
  motor.loopFOC();
  motor.move(target_position);
  command.run();
}
