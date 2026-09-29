#include <SimpleFOC.h>

// ============================================================
// ESP32-S3 + DRV8302 + Hoverboard BLDC
// Dual feedback (Custom Hall / MT6701)
// Current Sense + Serial CLI + Telemetry
//
// FIXES vs previous version (see "FIX" tags in CustomHallSensor):
//  1. Removed the private angle_prev that shadowed Sensor::angle_prev.
//     SimpleFOC computes the electrical angle from
//     sensor->getMechanicalAngle() (= base angle_prev), which was
//     never updated -> the field never rotated.
//  2. Hall angle is now referenced to the edge that was just crossed
//     (direction aware) and to the sector centre at rest.
//  3. Zero-velocity timeout raised from 40 ms to 150 ms.
//  4. POSITION: CMD:SP_POS:<deg> is converted into a multi-turn
//     motor.target near the current turn, limited to the 0..360 deg
//     window (see positionCommandToTarget()).
//  5. ALIGNMENT: low alignment voltage (ALIGN_VOLTAGE) and optional fixed
//     zero electric angle / direction (USE_FIXED_ALIGN) to avoid the
//     alignment current peak.
//  Telemetry format and serial commands are UNCHANGED.
// ============================================================

// ---------------- DRV8302 PWM pins ----------------
#define INL_A   15
#define INH_A   16
#define INL_B   6
#define INH_B   7
#define INL_C   4
#define INH_C   5
#define EN_DRV  14

// ---------------- DRV8302 control pins ----------------
#define MPWM    40
#define MOC     41
#define DC_CAL  39
#define GAIN    37
#define NFAULT  42
#define NOCTW   38

// ---------------- DRV8302 analog sense pins ----------------
#define A_SENSE_PIN    1
#define B_SENSE_PIN    2
#define C_SENSE_PIN    3
#define PVDD_SENSE_PIN 17
#define NTC_PIN        13
#define SO1_PIN        8
#define SO2_PIN        9

// ---------------- Motor profile ----------------
#define CFG_REDUCER_RATIO 1.0f

// ---------------- Hall sensors ----------------
#define HALL_A  10
#define HALL_B  11
#define HALL_C  12

// ---------------- MT6701 magnetic encoder ----------------
#define MAG_DO  48
#define MAG_CLK 21
#define MAG_CSN 18

static constexpr int   MAG_COUNTS_PER_REV = 16384;
static constexpr float MAG_TWO_PI = 6.28318530718f;

#define MOTOR_POLE_PAIRS 15

// Hall velocity timeout (us): no edge for this long -> velocity = 0
static constexpr float HALL_TIMEOUT_US = 150000.0f;

// ---------------- Position window (rad): 0 .. 360 deg ----------------
static constexpr float POS_MIN_RAD = 0.0f;
static constexpr float POS_MAX_RAD = 6.28318530718f;

// true : take the shortest way to the commanded angle (may cross 0/360)
// false: never cross 0/360, always move inside the window
static constexpr bool POS_SHORTEST_PATH = true;

// ---------------- Position shaping (anti-vibration / anti-overshoot) ----
static constexpr float POS_SLEW_RAD_S   = 6.0f;     // max speed of the position target (~57 rpm)
static constexpr float POS_DEADBAND_RAD = 0.05f;    // ~+-2.9 deg: must be > half a Hall sector (2 deg mech)

// ---------------- Setpoint slew rates (anti-overshoot) ----------------
static constexpr float VEL_SLEW_RAD_S2 = 40.0f;   // rad/s per second
static constexpr float TOR_SLEW_A_S     = 10.0f;  // A per second

// ---------------- Alignment ----------------
// Voltage applied during alignment (current ~ V / R, no current control
// there). Start low and raise in 0.5 V steps until initFOC() is reliable.
static constexpr float ALIGN_VOLTAGE = 2.0f;

// Step 1: keep false, boot a few times and read the printed
//         "zero_electric_angle" / "sensor_direction".
// Step 2: copy them below and set USE_FIXED_ALIGN = true -> the rotor no
//         longer moves at boot (no alignment current at all).
static constexpr bool      USE_FIXED_ALIGN      = false;
static constexpr float     HALL_ZERO_ELEC_ANGLE = 0.0f;         // printed value
static constexpr Direction HALL_SENSOR_DIR      = Direction::CW; // CW or CCW

// ============================================================
// Feedback selection
// ============================================================

enum FeedbackSys
{
  FBK_HALL,
  FBK_MAG
};

FeedbackSys fbk_sys = FBK_HALL;


// ============================================================
// Custom Hall Sensor
// ============================================================

class CustomHallSensor : public Sensor
{
  public:

    CustomHallSensor(int pp)
    {
      pole_pairs = pp;
    }

    void init() override
    {
      pinMode(HALL_A, INPUT_PULLUP);
      pinMode(HALL_B, INPUT_PULLUP);
      pinMode(HALL_C, INPUT_PULLUP);

      uint8_t hA = digitalRead(HALL_A);
      uint8_t hB = digitalRead(HALL_B);
      uint8_t hC = digitalRead(HALL_C);

      state = (hC << 2) | (hB << 1) | hA;

      int start_sector = state_to_sector[state];

      accumulated_elec_angle =
        (start_sector != -1) ? start_sector * 60.0f : 0.0f;

      int64_t now = micros();

      last_edge_time_us = now;
      pending_since_us  = now;
      pending_state     = state;

      have_pending   = false;
      edge_period_us = 0.0f;
      direction      = 0;
      have_edge      = false;

      // FIX: initialise the BASE class members so that the first update()
      // doesn't see a false jump / false full rotation.
      // At rest the estimate is the sector centre (+30 deg elec).
      float mech_deg = (accumulated_elec_angle + 30.0f) / pole_pairs;
      angle_prev     = mech_deg * DEG_TO_RAD;   // Sensor::angle_prev
      full_rotations = 0;                        // Sensor::full_rotations
      custom_angle   = angle_prev;
      custom_velocity = 0.0f;

      Sensor::init();

      // Sensor::init() calls update()/getAngle() itself; make sure we end
      // in a clean, consistent state.
      full_rotations = 0;
      angle_prev     = mech_deg * DEG_TO_RAD;
      custom_angle   = angle_prev;
    }

    void update() override
    {
      uint8_t hA = digitalRead(HALL_A);
      uint8_t hB = digitalRead(HALL_B);
      uint8_t hC = digitalRead(HALL_C);

      uint8_t raw_state = (hC << 2) | (hB << 1) | hA;

      int64_t now_us = micros();

      float max_elec_angle = 360.0f * pole_pairs;

      // --------------------------------------------------------
      // Debounce and edge detection
      // --------------------------------------------------------

      if (raw_state != state)
      {
        if (!have_pending || pending_state != raw_state)
        {
          pending_state    = raw_state;
          pending_since_us = now_us;
          have_pending     = true;
        }
        else if (now_us - pending_since_us >= 150)
        {
          uint8_t new_state = raw_state;

          int8_t prev_sec = state_to_sector[state];
          int8_t new_sec  = state_to_sector[new_state];

          if (prev_sec != -1 && new_sec != -1)
          {
            int8_t delta = new_sec - prev_sec;

            if (delta > 3)  delta -= 6;
            if (delta < -3) delta += 6;

            accumulated_elec_angle += delta * 60.0f;

            if (delta == 1 || delta == -1)
            {
              float period = (float)(now_us - last_edge_time_us);

              int8_t new_dir = (delta > 0) ? 1 : -1;

              if (direction != 0 && new_dir != direction)
              {
                // Reversal (rotor chattering on a Hall boundary):
                // no valid speed information, avoid false velocity spikes
                direction = new_dir;
                have_edge = false;
                edge_period_us = 0.0f;
              }
              else if (period > 300.0f && period < 500000.0f)
              {
                if (edge_period_us <= 0.0f)
                  edge_period_us = period;
                else
                  edge_period_us += 0.3f * (period - edge_period_us);

                direction = (delta > 0) ? 1 : -1;
                have_edge = true;
              }
              else
              {
                // Edge too fast (glitch) or too slow (restart from rest)
                direction = (delta > 0) ? 1 : -1;
                have_edge = false;
                edge_period_us = 0.0f;
              }
            }
            else if (delta != 0)
            {
              direction = (delta > 0) ? 1 : -1;
              have_edge = false;
              edge_period_us = 0.0f;
            }
          }

          state = new_state;
          last_edge_time_us = now_us;
          have_pending = false;
        }
      }
      else
      {
        have_pending = false;
      }

      // --------------------------------------------------------
      // Keep sector angle inside valid range
      // --------------------------------------------------------

      accumulated_elec_angle = fmodf(accumulated_elec_angle, max_elec_angle);

      if (accumulated_elec_angle < 0.0f)
        accumulated_elec_angle += max_elec_angle;

      float dt_since_us = (float)(now_us - last_edge_time_us);

      // --------------------------------------------------------
      // Timeout -> no valid speed information
      // --------------------------------------------------------

      if (dt_since_us > HALL_TIMEOUT_US ||
          (have_edge && dt_since_us > edge_period_us * 1.5f + 1000.0f))
      {
        have_edge = false;
        edge_period_us = 0.0f;
      }

      // --------------------------------------------------------
      // FIX: direction-aware angle estimate
      //
      // accumulated_elec_angle = start of the current sector.
      //  - moving forward: we entered through the sector start, so the
      //    rotor is at start + frac*60
      //  - moving backward: we entered through the sector end, so the
      //    rotor is at start + 60 - frac*60
      //  - at rest (no speed info): sector centre
      // --------------------------------------------------------

      float interpolated_angle;

      if (have_edge && edge_period_us > 0.0f)
      {
        float frac = dt_since_us / edge_period_us;

        if (frac > 1.0f)
          frac = 1.0f;

        if (direction > 0)
          interpolated_angle = accumulated_elec_angle + frac * 60.0f;
        else
          interpolated_angle = accumulated_elec_angle + 60.0f - frac * 60.0f;
      }
      else
      {
        interpolated_angle = accumulated_elec_angle + 30.0f;
      }

      interpolated_angle = fmodf(interpolated_angle, max_elec_angle);

      if (interpolated_angle < 0.0f)
        interpolated_angle += max_elec_angle;

      // --------------------------------------------------------
      // Electrical -> mechanical angle
      // --------------------------------------------------------

      float mech_angle_deg = interpolated_angle / pole_pairs;

      float current_angle_rad = mech_angle_deg * DEG_TO_RAD;

      // --------------------------------------------------------
      // Multi-turn angle
      // (angle_prev and full_rotations are the Sensor base members)
      // --------------------------------------------------------

      float d_angle = current_angle_rad - angle_prev;

      if (fabsf(d_angle) > (0.8f * _2PI))
      {
        full_rotations += (d_angle < 0) ? 1 : -1;
      }

      angle_prev = current_angle_rad;   // FIX: this is what
                                        // getMechanicalAngle() returns and
                                        // what the electrical angle uses

      custom_angle = (float)full_rotations * _2PI + current_angle_rad;

      // --------------------------------------------------------
      // Velocity
      // --------------------------------------------------------

      if (!have_edge || edge_period_us <= 0.0f)
      {
        custom_velocity = 0.0f;
      }
      else
      {
        float rpm =
          (float)direction *
          (10000000.0f / (edge_period_us * (float)pole_pairs));

        custom_velocity = rpm * (PI / 30.0f);
      }
    }

    float getAngle() override
    {
      return custom_angle;
    }

    float getVelocity() override
    {
      return custom_velocity;
    }

    float getSensorAngle() override
    {
      return angle_prev;
    }

  private:

    int pole_pairs;

    uint8_t state = 0;

    int8_t state_to_sector[8] = { -1, 0, 2, 1, 4, 5, 3, -1 };

    uint8_t pending_state = 0;

    int64_t pending_since_us  = 0;
    int64_t last_edge_time_us = 0;

    bool have_pending = false;
    bool have_edge    = false;

    float edge_period_us = 0.0f;

    int8_t direction = 0;

    float accumulated_elec_angle = 0.0f;

    // NOTE: no private angle_prev here any more (it shadowed Sensor::angle_prev)
    float custom_angle    = 0.0f;
    float custom_velocity = 0.0f;
};


// ============================================================
// Motor / Driver / Current Sense
// ============================================================

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);

BLDCDriver6PWM driver = BLDCDriver6PWM(
  INH_A, INL_A,
  INH_B, INL_B,
  INH_C, INL_C
);

// 0.01 Ohm shunt, 10x gain, SO1 = phase A, SO2 = phase B
InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 10.0f, SO1_PIN, SO2_PIN);

CustomHallSensor hall_sensor = CustomHallSensor(MOTOR_POLE_PAIRS);


// ============================================================
// MT6701 raw SSI
// ============================================================

uint32_t MT6701_read_raw()
{
  uint32_t raw = 0;

  digitalWrite(MAG_CSN, LOW);

  for (int i = 0; i < 24; i++)
  {
    digitalWrite(MAG_CLK, HIGH);
    delayMicroseconds(1);

    raw <<= 1;
    raw |= digitalRead(MAG_DO);

    digitalWrite(MAG_CLK, LOW);
    delayMicroseconds(1);
  }

  digitalWrite(MAG_CSN, HIGH);

  return raw;
}


float MT6701_readAngle()
{
  uint32_t raw = MT6701_read_raw();

  uint16_t counts = (raw >> 10) & 0x3FFF;

  return ((float)counts / (float)MAG_COUNTS_PER_REV) * MAG_TWO_PI;
}


void MT6701_initPins()
{
  pinMode(MAG_DO, INPUT);
  pinMode(MAG_CLK, OUTPUT);
  pinMode(MAG_CSN, OUTPUT);

  digitalWrite(MAG_CLK, LOW);
  digitalWrite(MAG_CSN, HIGH);
}


GenericSensor mag_sensor = GenericSensor(MT6701_readAngle, MT6701_initPins);


// ============================================================
// SimpleFOC Commander
// ============================================================

Commander command = Commander(Serial);

void onMotor(char* cmd)
{
  command.motor(&motor, cmd);
}


// ============================================================
// DRV8302 sensor parameters
// ============================================================

static constexpr float VOLTAGE_GAIN = 11.0f;
static constexpr float V_OFFSET     = 1.1f;

static constexpr float NTC_B        = 3380.0f;
static constexpr float NTC_T0       = 298.15f;
static constexpr float NTC_OFFSET   = 9.0f;

float last_temperature = 0.0f;

uint32_t last_ntc_read_ms = 0;


// ============================================================
// NTC
// ============================================================

float readNTCTemperature(int ntc_raw)
{
  float ratio = (float)ntc_raw / (4095.0f - (float)ntc_raw);

  return
    1.0f / (1.0f / NTC_T0 + (1.0f / NTC_B) * logf(ratio))
    - 273.15f
    - NTC_OFFSET;
}


// ============================================================
// PVDD
// ============================================================

float getPVDD(int pvdd_raw)
{
  return (float)pvdd_raw * 3.3f / 4095.0f * VOLTAGE_GAIN + V_OFFSET;
}


// ============================================================
// Reducer angle
// ============================================================

bool  reducer_initialized = false;
float reducer_last_angle  = 0.0f;
float reducer_accum       = 0.0f;


float updateReducerAngle(float motor_angle_deg)
{
  if (!reducer_initialized)
  {
    reducer_last_angle  = motor_angle_deg;
    reducer_accum       = motor_angle_deg;
    reducer_initialized = true;

    return reducer_accum;
  }

  float delta = fmodf(motor_angle_deg - reducer_last_angle, 360.0f);

  if (delta > 180.0f)  delta -= 360.0f;
  if (delta < -180.0f) delta += 360.0f;

  reducer_accum += delta / CFG_REDUCER_RATIO;

  reducer_last_angle = motor_angle_deg;

  return reducer_accum;
}


// ============================================================
// Telemetry (format UNCHANGED)
// ============================================================

bool     debug_mode_enabled    = true;
uint32_t telemetry_interval_ms = 50;
uint32_t last_telemetry_ms     = 0;


void printTelemetry()
{
  if (!debug_mode_enabled)
    return;

  uint32_t now = millis();

  if (now - last_telemetry_ms < telemetry_interval_ms)
    return;

  last_telemetry_ms = now;

  uint8_t nfault = digitalRead(NFAULT);
  uint8_t noctw  = digitalRead(NOCTW);

  uint16_t a_val = (uint16_t)(analogRead(A_SENSE_PIN) * VOLTAGE_GAIN);
  uint16_t b_val = (uint16_t)(analogRead(B_SENSE_PIN) * VOLTAGE_GAIN);
  uint16_t c_val = (uint16_t)(analogRead(C_SENSE_PIN) * VOLTAGE_GAIN);

  float pvdd = getPVDD(analogRead(PVDD_SENSE_PIN));

  if (now - last_ntc_read_ms > 500)
  {
    int raw_ntc = analogRead(NTC_PIN);

    last_temperature = readNTCTemperature(raw_ntc);

    last_ntc_read_ms = now;
  }

  // FOC currents
  float i_q = motor.current.q;
  float i_d = motor.current.d;

  // 1. Get raw continuous angle for calculations
  float raw_angle_deg = motor.shaft_angle * RAD_TO_DEG;

  // 2. Update reducer logic using the continuous angle to prevent delta glitches
  float raw_reducer_angle = updateReducerAngle(raw_angle_deg);

  // 3. Clamp Motor Angle (0 to 360) for Telemetry
  float angle_deg_telemetry = fmodf(raw_angle_deg, 360.0f);
  if (angle_deg_telemetry < 0.0f) {
      angle_deg_telemetry += 360.0f;
  }

  // 4. Clamp Reducer Angle (0 to 360) for Telemetry
  float reducer_angle_telemetry = fmodf(raw_reducer_angle, 360.0f);
  if (reducer_angle_telemetry < 0.0f) {
      reducer_angle_telemetry += 360.0f;
  }

  float rpm = motor.shaft_velocity * 9.5493f;
  float reducer_rpm = rpm / CFG_REDUCER_RATIO;

  // CSV telemetry
  Serial.print(1);                           Serial.print(',');
  Serial.print(pvdd, 3);                     Serial.print(',');
  Serial.print(i_q, 3);                      Serial.print(',');
  Serial.print(i_d, 3);                      Serial.print(',');
  Serial.print(-(i_q + i_d), 3);             Serial.print(',');
  Serial.print(a_val);                       Serial.print(',');
  Serial.print(b_val);                       Serial.print(',');
  Serial.print(c_val);                       Serial.print(',');
  Serial.print(nfault);                      Serial.print(',');
  Serial.print(noctw);                       Serial.print(',');
  Serial.print(last_temperature, 2);         Serial.print(',');
  Serial.print(angle_deg_telemetry, 2);      Serial.print(','); // Sent clamped
  Serial.print(reducer_angle_telemetry, 2);  Serial.print(','); // Sent clamped
  Serial.print(rpm, 2);                      Serial.print(',');
  Serial.println(reducer_rpm, 2);
}

// ============================================================
// Setpoint shaping (slew limit) + position clamp
//
// target_cmd : what the user asked for (velocity / torque)
// motor.target : what the controller actually gets (ramped)
//
// In ANGLE mode CMD:SP_POS sets pos_cmd (multi-turn target, see
// positionCommandToTarget()); motor.target follows it with a ramp and a
// small dead band around the current position.
// ============================================================

float    target_cmd     = 0.0f;
float    pos_cmd        = 0.0f;   // commanded position (multi-turn, rad)
uint32_t last_shape_us  = 0;

static float slewToward(float cur, float tgt, float max_step)
{
  float d = tgt - cur;

  if (d >  max_step) return cur + max_step;
  if (d < -max_step) return cur - max_step;

  return tgt;
}

void applyTargetShaping()
{
  uint32_t now = micros();

  float dt = (now - last_shape_us) * 1e-6f;

  last_shape_us = now;

  if (dt > 0.05f)
    dt = 0.05f;

  if (motor.controller == MotionControlType::velocity)
  {
    motor.target = slewToward(motor.target, target_cmd, VEL_SLEW_RAD_S2 * dt);
  }
  else if (motor.controller == MotionControlType::torque)
  {
    motor.target = slewToward(motor.target, target_cmd, TOR_SLEW_A_S * dt);
  }
  else if (motor.controller == MotionControlType::angle)
  {
    float err = pos_cmd - motor.shaft_angle;

    if (fabsf(err) < POS_DEADBAND_RAD)
      motor.target = motor.shaft_angle;   // hold, don't chase micro-errors
    else
      motor.target = slewToward(motor.target, pos_cmd, POS_SLEW_RAD_S * dt);
  }
}

// Switch control mode starting from the current state (no jump)
void setControlMode(MotionControlType t)
{
  if (motor.controller != t)
  {
    if (t == MotionControlType::velocity)
    {
      motor.target = motor.shaft_velocity;
      target_cmd   = 0.0f;
    }
    else if (t == MotionControlType::torque)
    {
      motor.target = motor.current.q;
      target_cmd   = 0.0f;
    }
    else if (t == MotionControlType::angle)
    {
      // hold the current (multi-turn) position
      motor.target = motor.shaft_angle;
      pos_cmd      = motor.shaft_angle;
    }
  }

  motor.updateMotionControlType(t);
}

// Convert a commanded angle [rad] inside the position window into a
// multi-turn motor.target close to the current turn.
float positionCommandToTarget(float tgt_rad)
{
  // limit to the 0..360 deg window
  tgt_rad = constrain(tgt_rad, POS_MIN_RAD, POS_MAX_RAD);

  float cur = motor.shaft_angle;

  // wrapped current angle (0..2pi)
  float cur_w = fmodf(cur, _2PI);
  if (cur_w < 0.0f)
    cur_w += _2PI;

  float diff = tgt_rad - cur_w;

  if (POS_SHORTEST_PATH)
  {
    diff = fmodf(diff, _2PI);

    if (diff >  PI) diff -= _2PI;
    if (diff < -PI) diff += _2PI;
  }

  return cur + diff;
}


// ============================================================
// Differential feedback switching
// ============================================================

void switchFeedback(FeedbackSys sys)
{
  if (sys == fbk_sys)
    return;

  // Stop before changing sensor
  motor.target = 0.0f;

  motor.disable();

  delay(50);

  fbk_sys = sys;

  if (fbk_sys == FBK_MAG)
  {
    motor.linkSensor(&mag_sensor);
    Serial.println("FBK -> MAG_ENCODER");
  }
  else
  {
    motor.linkSensor(&hall_sensor);
    Serial.println("FBK -> HALL_SENSOR");
  }

  // Re-run FOC alignment with the new sensor
  motor.enable();

  bool ok = motor.initFOC();

  pos_cmd = motor.shaft_angle;   // new sensor -> hold its current position

  if (ok)
    Serial.println("FOC alignment OK");
  else
    Serial.println("FOC alignment FAILED");
}


// ============================================================
// Serial command parser (UNCHANGED)
//
// CMD:FBK:MAG_ENCODER / CMD:FBK:HALL_SENSOR
// CMD:CTRL:TORQUE / CMD:CTRL:VELOCITY / CMD:CTRL:ANGLE
// CMD:IDLE / CMD:ALIGN / CMD:RUN / CMD:BRAKE / CMD:STOP
// CMD:PID_VEL_KP:<v>_KI:<v>_KD:<v>
// CMD:PID_POS_KP:<v>_KI:<v>_KD:<v>
// CMD:PID_TORQUE_KP:<v>_KI:<v>_KD:<v>
// CMD:PID_FLUX_KP:<v>_KI:<v>_KD:<v>
// CMD:SP_POS:<rad> / CMD:SP_VEL:<rad/s> / CMD:SP_TOR:<A>
// CMD:DEBUG_ON / CMD:DEBUG_OFF
// ============================================================

void handleSerialCommands()
{
  static char rxbuf[64];
  static uint8_t rxlen = 0;

  while (Serial.available())
  {
    char c = Serial.read();

    // End of command
    if (c == '\n' || c == '\r')
    {
      if (rxlen == 0)
        continue;

      rxbuf[rxlen] = '\0';

      // Text before the first ':' is the prefix ("CMD")
      char* cmd = strchr(rxbuf, ':');

      if (cmd != NULL)
      {
        cmd++;

        // ---------------- Feedback ----------------
        if (strncmp(cmd, "FBK:MAG_ENCODER", 15) == 0)
        {
          switchFeedback(FBK_MAG);
        }
        else if (strncmp(cmd, "FBK:HALL_SENSOR", 15) == 0)
        {
          switchFeedback(FBK_HALL);
        }

        // ---------------- Controller selection ----------------
        else if (strncmp(cmd, "CTRL:TORQUE", 11) == 0)
        {
          setControlMode(MotionControlType::torque);
          Serial.println("Controller -> TORQUE");
        }
        else if (strncmp(cmd, "CTRL:VELOCITY", 13) == 0)
        {
          setControlMode(MotionControlType::velocity);
          Serial.println("Controller -> VELOCITY");
        }
        else if (strncmp(cmd, "CTRL:ANGLE", 10) == 0)
        {
          setControlMode(MotionControlType::angle);
          Serial.println("Controller -> ANGLE");
        }

        // ---------------- IDLE ----------------
        else if (strcmp(cmd, "IDLE") == 0)
        {
          motor.disable();
          Serial.println("Motor IDLE");
        }

        // ---------------- ALIGN ----------------
        else if (strcmp(cmd, "ALIGN") == 0)
        {
          motor.enable();

          bool ok = motor.initFOC();

          if (ok)
            Serial.println("FOC ALIGN OK");
          else
            Serial.println("FOC ALIGN FAILED");
        }

        // ---------------- RUN ----------------
        else if (strncmp(cmd, "RUN", 3) == 0)
        {
          motor.enable();
          Serial.println("Running");
        }

        // ---------------- BRAKE / STOP ----------------
        else if (strcmp(cmd, "BRAKE") == 0 || strcmp(cmd, "STOP") == 0)
        {
          // Original behavior preserved: target is NOT reset here.
          motor.disable();
          Serial.println("Motor stopped");
        }

        // ---------------- PID commands ----------------
        else if (strncmp(cmd, "PID_", 4) == 0)
        {
          char prefix[10];

          float Kp = 0.0f;
          float Ki = 0.0f;
          float Kd = 0.0f;

          if (sscanf(cmd, "PID_%9[^_]_KP:%f_KI:%f_KD:%f",
                     prefix, &Kp, &Ki, &Kd) == 4)
          {
            if (strcmp(prefix, "VEL") == 0)
            {
              motor.PID_velocity.P = Kp;
              motor.PID_velocity.I = Ki;
              motor.PID_velocity.D = Kd;
            }
            else if (strcmp(prefix, "POS") == 0)
            {
              motor.P_angle.P = Kp;
              motor.P_angle.I = Ki;
              motor.P_angle.D = Kd;
            }
            else if (strcmp(prefix, "TORQUE") == 0 ||
                     strcmp(prefix, "FLUX") == 0)
            {
              motor.PID_current_q.P = Kp;
              motor.PID_current_q.I = Ki;
              motor.PID_current_q.D = Kd;

              motor.PID_current_d.P = Kp;
              motor.PID_current_d.I = Ki;
              motor.PID_current_d.D = Kd;
            }

            Serial.print(prefix);
            Serial.print(" PID updated: Kp=");
            Serial.print(Kp, 4);
            Serial.print(" Ki=");
            Serial.print(Ki, 4);
            Serial.print(" Kd=");
            Serial.println(Kd, 4);
          }
        }

        // ---------------- Setpoints ----------------
        else if (strncmp(cmd, "SP_", 3) == 0)
        {
          float target = 0.0f;

          if (sscanf(cmd, "SP_POS:%f", &target) == 1)
          {
            setControlMode(MotionControlType::angle);

            // Input in degrees -> radians, limited to the 0..360 window,
            // then converted into a multi-turn target near the current
            // turn (this is what the angle controller really needs).
            float tgt_rad = target * (PI / 180.0f);

            pos_cmd = positionCommandToTarget(tgt_rad);   // motor.target follows with a ramp

            Serial.print("POS target updated [deg]: ");
            Serial.print(target, 4);
            Serial.print(" -> [rad]: ");
            Serial.println(pos_cmd, 4);
          }
          else if (sscanf(cmd, "SP_VEL:%f", &target) == 1)
          {
            setControlMode(MotionControlType::velocity);

            // Convertir la entrada de RPM a rad/s para SimpleFOC
            target_cmd = target * (PI / 30.0f);

            Serial.print("VEL target updated [RPM]: ");
            Serial.print(target, 4);
            Serial.print(" -> [rad/s]: ");
            Serial.println(target_cmd, 4);
          }
          else if (sscanf(cmd, "SP_TOR:%f", &target) == 1)
          {
            setControlMode(MotionControlType::torque);

            target_cmd = target;   // motor.target follows with a ramp

            Serial.print("TOR target updated: ");
            Serial.println(target, 4);
          }
        }

        // ---------------- Debug ----------------
        else if (strcmp(cmd, "DEBUG_ON") == 0)
        {
          debug_mode_enabled = true;
          Serial.println("Debug ON");
        }
        else if (strcmp(cmd, "DEBUG_OFF") == 0)
        {
          debug_mode_enabled = false;
          Serial.println("Debug OFF");
        }
      }

      rxlen = 0;
    }

    // Add character to buffer
    else if (rxlen < sizeof(rxbuf) - 1)
    {
      rxbuf[rxlen++] = c;
    }
  }
}


// ============================================================
// DRV8302 power-up sequence
// ============================================================

bool powerUpDRV8302()
{
  pinMode(EN_DRV, OUTPUT);
  digitalWrite(EN_DRV, LOW);

  Serial.println("Powering up DRV8302...");

  delay(1000);

  digitalWrite(EN_DRV, HIGH);

  delay(1000);

  if (digitalRead(NFAULT) == LOW)
  {
    Serial.println("ERROR: DRV8302 FAULT ERROR");
    return false;
  }

  Serial.println("DRV8302 POWERED UP");

  return true;
}


// ============================================================
// Setup
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println("======================================");
  Serial.println(" ESP32-S3 + DRV8302 + Hoverboard BLDC");
  Serial.println(" Dual Feedback + Telemetry + Serial CLI");
  Serial.println("======================================");

  // ----------------------------------------------------------
  // DRV8302 control pins
  // ----------------------------------------------------------

  pinMode(MPWM, OUTPUT);    digitalWrite(MPWM, LOW);
  pinMode(MOC, OUTPUT);     digitalWrite(MOC, LOW);
  pinMode(DC_CAL, OUTPUT);  digitalWrite(DC_CAL, LOW);
  pinMode(GAIN, OUTPUT);    digitalWrite(GAIN, LOW);

  pinMode(NFAULT, INPUT);
  pinMode(NOCTW, INPUT);

  // ----------------------------------------------------------
  // ADC configuration
  // ----------------------------------------------------------

  analogReadResolution(12);

  analogSetPinAttenuation(A_SENSE_PIN, ADC_11db);
  analogSetPinAttenuation(B_SENSE_PIN, ADC_11db);
  analogSetPinAttenuation(C_SENSE_PIN, ADC_11db);
  analogSetPinAttenuation(PVDD_SENSE_PIN, ADC_11db);
  analogSetPinAttenuation(NTC_PIN, ADC_11db);

  // ----------------------------------------------------------
  // Sensors
  // ----------------------------------------------------------

  hall_sensor.init();
  mag_sensor.init();

  motor.linkSensor(&hall_sensor);

  // ----------------------------------------------------------
  // DRV8302 power-up
  // ----------------------------------------------------------

  if (!powerUpDRV8302())
  {
    while (1)
      delay(1000);
  }

  // ----------------------------------------------------------
  // Driver
  // ----------------------------------------------------------

  driver.voltage_power_supply = 16.0f;
  driver.voltage_limit        = 12.0f;
  driver.pwm_frequency        = 20000;
  driver.dead_zone            = 0.02f;

  if (!driver.init())
  {
    Serial.println("ERROR: DRV8302 driver initialization FAILED!");

    while (1)
      delay(1000);
  }

  motor.linkDriver(&driver);

  // ----------------------------------------------------------
  // Current sense
  // ----------------------------------------------------------

  bool current_sense_ok = current_sense.init();

  if (current_sense_ok)
  {
    // If the motor runs backwards / wobbles after the angle fix,
    // set this to false once to let SimpleFOC detect gain signs and
    // phase mapping automatically.
    current_sense.skip_align = true;

    motor.linkCurrentSense(&current_sense);

    Serial.println("Current sense initialized successfully.");
  }
  else
  {
    Serial.println("ERROR: Current sense init failed!");
  }

  // ----------------------------------------------------------
  // FOC modulation / controller
  // ----------------------------------------------------------

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motor.controller = MotionControlType::angle;

  // Torque control on q-axis current (uses current sense)
  motor.torque_controller = TorqueControlType::foc_current;

  motor.current_limit = 3.0f;

  // Faster current filter (was 0.01 s: its lag causes overshoot)
  motor.LPF_current_q.Tf = 0.004f;
  motor.LPF_current_d.Tf = 0.004f;

  // Softer inner current loop (SimpleFOC default P=3, I=300 is aggressive
  // at 16 V). Tune live with CMD:PID_TORQUE_KP:<v>_KI:<v>_KD:0
  motor.PID_current_q.P = 1.5f;
  motor.PID_current_q.I = 100.0f;
  motor.PID_current_q.D = 0.0f;

  motor.PID_current_d.P = 1.5f;
  motor.PID_current_d.I = 100.0f;
  motor.PID_current_d.D = 0.0f;

  // ----------------------------------------------------------
  // VELOCITY LOOP  (output = current target in A)
  //
  // I lowered from 10 to 5 for first tests with the Hall sensor
  // (low-resolution velocity). Raise it again with
  // CMD:PID_VEL_KP:0.2_KI:10_KD:0 once the wheel turns properly.
  // ----------------------------------------------------------

  motor.PID_velocity.P = 0.10f;
  motor.PID_velocity.I = 0.48f;
  motor.PID_velocity.D = 0.0f;

  motor.PID_velocity.output_ramp = 100.0f;   // A/s max change of current setpoint

  motor.LPF_velocity.Tf = 0.01f;

  // ----------------------------------------------------------
  // POSITION LOOP (position -> velocity -> current)
  // ----------------------------------------------------------

  motor.P_angle.P = 6.93f;
  motor.P_angle.I = 0.4f;
  motor.P_angle.D = 0.0f;

  // ----------------------------------------------------------
  // FOC alignment / limits
  // ----------------------------------------------------------

  motor.voltage_sensor_align = ALIGN_VOLTAGE;   // low current alignment

  // 20 rad/s ~= 191 RPM
  motor.velocity_limit = 20.0f;

  motor.voltage_limit = 12.0f;

  // ----------------------------------------------------------
  // Monitoring
  // ----------------------------------------------------------

  motor.useMonitoring(Serial);

  motor.monitor_downsample = 0;

  // ----------------------------------------------------------
  // Initialize motor
  // ----------------------------------------------------------

  motor.init();

  Serial.println("Motor initialized.");

  // ----------------------------------------------------------
  // FOC alignment retries
  // ----------------------------------------------------------

  bool foc_ok = false;

  const int FOC_INIT_ATTEMPTS = 5;

  for (int attempt = 1; attempt <= FOC_INIT_ATTEMPTS && !foc_ok; attempt++)
  {
    if (attempt > 1)
    {
      Serial.print("Retrying FOC alignment, attempt ");
      Serial.print(attempt);
      Serial.print("/");
      Serial.println(FOC_INIT_ATTEMPTS);

      delay(300);
    }

    if (USE_FIXED_ALIGN)
    {
      // Presetting these makes initFOC() skip the alignment movement
      motor.zero_electric_angle = HALL_ZERO_ELEC_ANGLE;
      motor.sensor_direction    = HALL_SENSOR_DIR;
    }

    foc_ok = motor.initFOC();
  }

  if (!foc_ok)
  {
    Serial.println();
    Serial.println("ERROR: FOC initialization FAILED after retries!");
    Serial.println("Check: phase wiring continuity, rotor spins freely by hand,");
    Serial.println("DRV8302 power, motor pole pairs.");

    while (1)
      delay(1000);
  }

  Serial.println("FOC initialization OK.");

  // Values to copy into HALL_ZERO_ELEC_ANGLE / HALL_SENSOR_DIR
  Serial.print("zero_electric_angle: ");
  Serial.println(motor.zero_electric_angle, 4);
  Serial.print("sensor_direction: ");
  Serial.println((int)motor.sensor_direction);

  // ----------------------------------------------------------
  // Startup IDLE
  // ----------------------------------------------------------

  motor.target = motor.shaft_angle;
  pos_cmd      = motor.shaft_angle;   // hold position until SP_POS

  motor.disable();

  Serial.println("Startup complete. Motor is IDLE (disabled), holding current position.");
  Serial.println("Send CMD:RUN to enable.");
  Serial.println("Send CMD:SP_POS:<rad> to command position.");
  Serial.println("Send CMD:SP_VEL:<rad/s> to command velocity.");
  Serial.println("Send CMD:SP_TOR:<A> to command torque/current.");

  // ----------------------------------------------------------
  // Commander
  // ----------------------------------------------------------

  command.add('M', onMotor, "motor");
}


// ============================================================
// Main loop
// ============================================================

void loop()
{
  // Ramp setpoints / clamp position window
  applyTargetShaping();

  // FOC loop (sensor update, current control, PWM)
  motor.loopFOC();

  // Motion control:
  //  torque:   target current -> foc current controller
  //  velocity: target vel -> velocity PID -> current target
  //  angle:    target angle -> P -> velocity PID -> current target
  motor.move();

  handleSerialCommands();

  command.run();

  printTelemetry();
}
