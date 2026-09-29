#include <SimpleFOC.h>

// ============================================================
// ESP32-S3 + DRV8302
// ============================================================

// DRV8302 6-PWM pins
#define INL_A   15
#define INH_A   16

#define INL_B   6
#define INH_B   7

#define INL_C   4
#define INH_C   5

#define EN_DRV  14

// DRV8302 control pins
#define MPWM    40
#define MOC     41

// Hall sensors
#define HALL_A  10
#define HALL_B  11
#define HALL_C  12

// Motor
#define MOTOR_POLE_PAIRS 5

// ============================================================
// SimpleFOC objects
// ============================================================

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);

BLDCDriver6PWM driver = BLDCDriver6PWM(
  INH_A, INL_A,
  INH_B, INL_B,
  INH_C, INL_C,
  EN_DRV
);

HallSensor sensor = HallSensor(
  HALL_A,
  HALL_B,
  HALL_C,
  MOTOR_POLE_PAIRS
);

// Hall interrupt callbacks
void doA() {
  sensor.handleA();
}

void doB() {
  sensor.handleB();
}

void doC() {
  sensor.handleC();
}

// Commander
Commander command = Commander(Serial);

void onMotor(char* cmd) {
  command.motor(&motor, cmd);
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);
  delay(500);

  // ----------------------------------------------------------
  // DRV8302 control pins
  // ----------------------------------------------------------

  pinMode(MPWM, OUTPUT);
  digitalWrite(MPWM, LOW);

  pinMode(MOC, OUTPUT);
  digitalWrite(MOC, LOW);

  // ----------------------------------------------------------
  // Hall sensor
  // ----------------------------------------------------------

  sensor.pullup = Pullup::USE_EXTERN;

  sensor.init();

  sensor.enableInterrupts(
    doA,
    doB,
    doC
  );

  // ----------------------------------------------------------
  // Motor / sensor configuration
  // ----------------------------------------------------------

  motor.linkSensor(&sensor);

  // ----------------------------------------------------------
  // DRV8302 driver
  // ----------------------------------------------------------

  driver.voltage_power_supply = 12.0f;
  driver.voltage_limit = 12.0f;

  // ESP32-S3 SimpleFOC PWM
  driver.pwm_frequency = 20000;

  // Dead zone for complementary PWM
  driver.dead_zone = 0.02f;

  if (!driver.init()) {
    Serial.println("DRV8302 driver initialization FAILED!");
    while (1);
  }

  motor.linkDriver(&driver);

  // ----------------------------------------------------------
  // Motor configuration
  // ----------------------------------------------------------

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motor.controller = MotionControlType::torque;

  // Velocity PID
  motor.PID_velocity.P = 0.1f;
  motor.PID_velocity.I = 0.01f;
  motor.PID_velocity.D = 0.0f;

  // Velocity low-pass filter
  motor.LPF_velocity.Tf = 0.01f;

  // Position controller
  motor.P_angle.P = 20.0f;

  // Limits
  motor.velocity_limit = 50.0f;
  motor.voltage_limit = 12.0f;

  // ----------------------------------------------------------
  // Motor initialization
  // ----------------------------------------------------------

  motor.init();

  if (!motor.initFOC()) {
    Serial.println("FOC initialization FAILED!");
    while (1);
  }

  // ----------------------------------------------------------
  // Commander
  // ----------------------------------------------------------

  motor.useMonitoring(Serial);

  command.add('M', onMotor, "motor");

  Serial.println("=================================");
  Serial.println("ESP32-S3 + DRV8302 + Hall Sensors");
  Serial.println("SimpleFOC ready");
  Serial.println("=================================");

  delay(1000);

  // Initial torque target
  motor.target = 2.0f;
}

// ============================================================
// LOOP
// ============================================================

void loop() {

  // Run FOC
  motor.loopFOC();

  // Run motor controller
  motor.move();

  // Serial commander
  command.run();
}