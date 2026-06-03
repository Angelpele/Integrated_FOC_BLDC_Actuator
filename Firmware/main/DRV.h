//================================================================================================================//
//                                                 DRV 8302 & Peripherals                                         //
//================================================================================================================//
#ifndef DRV_H
#define DRV_H 


#include "esp32_S3.h"


enum DRV_Error {
    DRV_OK = 0,
    DRV_OVERVOLTAGE = -1,
    DRV_FAULT = -2,
    DRV_WARNING = -3,
    DRV_LATCHED = -4
};

struct AlignOffset {
    bool  done;
    float elec_offset;
};

typedef struct {
    float PVDD;
    float current1;
    float current2;
    uint16_t A;
    uint16_t B;
    uint16_t C;
    uint8_t NFAULT_raw;
    uint8_t NOCTW_raw;
    float temperature;
}DRV_data;


class DRV {
    public:
        int init();
        int calibration();
        DRV_data getdata();
        int fault_detection();

        float real_RPM(float angle_deg);
        float reducer_angle(float motor_angle);
        void setup_mcpwm();
        bool svpwm(float v_a, float v_b, float v_c);
        AlignOffset align(float angle);
        bool ramp(float angle, float init_freq, float obj_freq);
        bool sinusoidal_spin(float fbk_angle, float freq);
        bool foc_control(float fbk_angle);

        float position_loop(float fbk_angle_deg);
        float velocity_loop(float fbk_rpm);

        void request_foc_stop() { foc_stop = true; }
        void reset_foc() {
            i_d_filt = 0.0f; i_q_filt = 0.0f;
            errorq_prev = 0.0f; errord_prev = 0.0f;
            errorq_integral = 0.0f; errord_integral = 0.0f;
            foc_initialized = false;
            foc_stop = false;
        }
        void reset_pos_loop() {
            pos_error_integral = 0.0f;
            pos_error_prev = 0.0f;
            pos_error_filt = 0.0f;
            pos_initialized = false;
            pos_prev_time = 0.0f;
        }
        void reset_vel_loop() {
            vel_error_integral = 0.0f;
            vel_error_prev     = 0.0f;
            vel_initialized = false;
            vel_prev_time = 0.0f;
        }
        bool openloop_bemf(float target_freq, float duty);
        void brake();
        void idle();
        void test_working_trans();

        void set_override(bool state){
            fault_override = state;
            if (state)
                printf("DRV FAULT OVERRIDE ENABLED\n");
            else
                printf("DRV FAULT OVERRIDE DISABLED\n");
        }

        float get_last_rpm() const { return last_rpm; }
        float update_rpm (float angle){ last_rpm = real_RPM(angle); return last_rpm;}
        float getPVDD(int PVDD_raw){
            float R_frac = 11;  // 100k + 10k / 10k
            return (float)PVDD_raw * 3.3 / 4095 * R_frac + V_offset;
        }

        void set_MOC(bool state){
            gpio_set_level(MOC, state ? 1 : 0);
        }

        void set_spin_duration(float duration_s){ spin_duration = duration_s; }
        void set_PID_tuning(uint8_t parameter, float Kp, float Ki, float Kd){
            switch (parameter) {
                case 0: Vel_Kp = Kp; Vel_Ki = Ki; Vel_Kd = Kd; break;
                case 1: Pos_Kp = Kp; Pos_Ki = Ki; Pos_Kd = Kd; break;
                case 2: Iq_Kp  = Kp; Iq_Ki = Ki; Iq_Kd = Kd; break;
                case 3: Id_Kp  = Kp; Id_Ki = Ki; Id_Kd = Kd; break;
                default: break;
            }
        }
        void set_target_pos(float pos)       { target_pos = pos; }
        void set_target_vel(float vel)       { target_vel = vel; }
        void set_target_torque(float torque) { target_current = torque; }

        void reset_spin_timer() {
            spin_timer_started = false;
            spin_elapsed = 0.0f;
            theta_reset = true;
            phase_accumulator = 0.0f;     
            electrical_freq = 2.0f;        
            rpm_last_time = 0;
            ramp_active = false;
            align_time = 0.0f;
        }
        void set_ramp_duration(float duration_s){ ramp_duration = duration_s; }

        float reducer_RPM(float motor_RPM){ return motor_RPM / reducer_ratio; }

    private: 
        // DRV data
        uint16_t SO1_raw = 0;   //Output current 1
        uint16_t SO2_raw = 0;   //Output current 2
        uint16_t NFAULT_raw = 0;
        uint16_t NOCTW_raw = 0;
        int PVDD_raw = 0;
        float V_offset = 1.1f; // Voltage offset

        // DRV parameters
        float R_shunt = 0.01;
        float voltage_step = 3.3/4095;
        float overvoltage = 16;
        uint16_t SO1_offset = 2048; // Default to mid-range (1.65V)
        uint16_t SO2_offset = 2048;

        // Motor parameters (5010 750KV)
        static constexpr int MOTOR_POLE_PAIRS = 7;   // 14 poles
        static constexpr float MOTOR_RESISTANCE = 0.270f; // Ohms
        static constexpr int MOTOR_MAX_CURRENT = 40; // Amps
        static constexpr float reducer_ratio = 15.0f; // Reducer ratio

        // DRV current sensing
        float current_gain = 40.0; 

        //Voltage sensing
        float voltage_gain = 11.0f; // 100k + 10k / 10k

        // External sensors data
        int NTC_raw = 0;
        uint32_t last_ntc_read = 0;
        float last_temperature = 0.0f;
        
        // Fault handling
        uint16_t latch_counter = 0;
        bool fault_override = false;
        uint8_t nfault_glitch_count = 0; 

        // Alignment and offset
        bool aligned = false;
        float offset = 0.0;
        float align_time = 0.0;
        float ALIGN_DURATION = 0.5f;   // 1500 ms
        float ALIGN_DUTY = 30.0f;

        // Waveform generation
        uint32_t PWM_freq = 40000; // 40 kHz carrier frequency for MCPWM
        float phase_accumulator = 0;
        float modulation_index = 1.0f; // Amplitude (0.0 to 1.0) if higher -> overmodulation, 0.8 best for high speeds
        float electrical_freq = 2; // Motor Frequency (Hz), speed of the rotating field, higher than 0.5 to avoid deadzone
        float elec_offset = 0.0f; 
        float theta_reset = true;

        // Ramping control
        float ramp_rate = 50.0f; // How fast to change frequency (Hz/s)
        float ramp_active = false;
        float ramp_target_freq = 0.0f;
        float ramp_duration = .5f;
        float dt = 0.000050f; // Elapsed time (assuming 50us)

        // Spinning control
        float spin_elapsed = 0.0f;
        float spin_duration = 0.0f;
        float spin_start_time = 0.0f;
        bool spin_timer_started = false;

        // Reducer angle
        float last_angle = 0.0f;
        float reducer = 0.0f;
        bool initialized = false;

        // RPM calculation
        float   rpm_last_angle  = 0.0f;
        int64_t rpm_last_time   = 0;
        float   rpm_filtered    = 0.0f;
        float last_rpm = 0.0f;
        
        float CLOSED_LOOP_LPF = 1.0f;  // 1.0 = no filtering for mag
        float RPM_LPF      = 0.80f;    // 0.3 for fast speeds, 0.8 for slow speeds
        static constexpr float RPM_UNWRAP_THR     = 180.0f;
        static constexpr int64_t RPM_MIN_DT_US    = 100;    // guard against /0 on first call

        // Velocity tunning
        float Vel_Kp = 0.002f;
        float Vel_Ki = 0.008f;
        float Vel_Kd = 0.0f;
        float Vel_LPF = 0.05f;
        float target_vel = 0.0f;

        // Position tunning
        float Pos_Kp = 1.0f;
        float Pos_Ki = 0.0f;
        float Pos_Kd = 0.0f;
        float Pos_LPF = 0.5f;
        float target_pos = 0.0f;

        // Current tunning
        float Id_LPF = 0.2f;
        float Iq_LPF = 0.2f;
        float Iq_Kp  = 0.05f;   float Iq_Ki = 200.0f;   float Iq_Kd = 0.0f;
        float Id_Kp  = 0.05f;   float Id_Ki = 200.0f;   float Id_Kd = 0.0f;
        float i_d_filt = 0.0f;
        float i_q_filt = 0.0f;
        float target_current = 0.0f;
        float errorq_prev = 0.0f;
        float errorq_integral = 0.0f;
        float errord_prev = 0.0f;
        float errord_integral = 0.0f;
        float foc_prev_time = 0.0f;
        bool foc_initialized = false;
        bool just_aligned = true;
        bool foc_stop = false;

        // Position loop state
        float pos_error_integral = 0.0f;
        float pos_error_filt = 0.0f;
        float pos_error_prev = 0.0f;
        float pos_prev_time = 0.0f;
        bool pos_initialized = false;

        // Velocity loop state
        float vel_error_integral = 0.0f;
        float fbk_rpm_filt = 0.0f;
        float vel_error_prev = 0.0f;
        float vel_prev_time = 0.0f;
        bool vel_initialized = false;

        // Velocity loop output clamp (Amps)
        static constexpr float VEL_OUT_MAX = 10.0f;
        // Position loop output clamp (RPM)
        static constexpr float POS_OUT_MAX = 1000.0f;

    };

class NTC_sensor {
    public:
        float getTemperature(int NTC_raw){
            float ratio = (float)NTC_raw / (4095.0f - (float)NTC_raw);
            return 1.0f / (INV_T0 + INV_B * logf(ratio)) - 273.15f - offset;
        }
        void change_offset(int mod){offset += (float)mod;}

    private:
        static constexpr int B  = 3380;
        static constexpr float T0 = 298.15f;  // 25 + 273.15

        static constexpr float INV_T0 = 1.0f / T0;
        static constexpr float INV_B = 1.0f / (float)B;

        float offset = 9.0f;
};

#endif