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

enum DRV_MotionState {
    MOTION_IDLE,
    MOTION_ALIGNING,
    MOTION_RAMPING,
    MOTION_SPINNING,
    MOTION_RAMPING_DOWN
};

struct AlignOffset {
    bool  done;
    float elec_offset;
};

typedef struct {
    float PVDD;
    float current1;
    float current2;
    uint16_t A_raw;
    uint16_t B_raw;
    uint16_t C_raw;
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
        AlignOffset align(float angle);
        bool ramp(float angle, float init_freq, float obj_freq, float duration_s);
        bool spin(float angle, float freq, float duration_s);
        bool svpwm(float angle, float freq);
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

        float getPVDD(int PVDD_raw){
            float R_frac = 11;  // 100k + 10k / 10k
            return (float)PVDD_raw * 3.3 / 4095 * R_frac;
        }

        void set_MOC(bool state){
            gpio_set_level(MOC, state ? 1 : 0);
        }

        float reducer_RPM(float motor_RPM){
            return motor_RPM / reducer_ratio;
        }


    private: 
        // DRV data
        uint16_t SO1_raw = 0;   //Output current 1
        uint16_t SO2_raw = 0;   //Output current 2
        uint16_t NFAULT_raw = 0;
        uint16_t NOCTW_raw = 0;
        int PVDD_raw = 0;

        // DRV parameters
        float R_shunt = 0.01;
        float voltage_step = 3.3/4095;
        float overvoltage = 16;
        uint16_t SO1_offset = 2048; // Default to mid-range (1.65V)
        uint16_t SO2_offset = 2048;

        // Motor parameters (5010 750KV)
        static constexpr int MOTOR_POLE_PAIRS = 7;   // 14 poles
        static constexpr int MOTOR_RESISTANCE = 270; // mOhm
        static constexpr int MOTOR_MAX_CURRENT = 40; // Amps
        static constexpr float reducer_ratio = 15.0f; // Reducer ratio

        // DRV current sensing
        float current_gain = 40.0; 

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
        float ALIGN_DURATION = 1.5f;   // 1500 ms
        float ALIGN_DUTY = 15.0f;

        // Waveform generation
        uint32_t PWM_freq = 20000; // 20 kHz carrier frequency for MCPWM
        float phase_accumulator = 0;
        float modulation_index = 1; // Amplitude (0.0 to 1.0) if higher -> overmodulation
        float electrical_freq = 2; // Motor Frequency (Hz), speed of the rotating field, higher than 0.5 to avoid deadzone
        float elec_offset = 0.0f; 

        // Ramping control
        float ramp_rate = 100.0f; // How fast to change frequency (Hz/s)
        float ramp_active = false;
        float ramp_target_freq = 0.0f;
        float ramp_duration = 0.2f;
        float dt = 0.000050f; // Elapsed time (assuming 50us)

        // Spinning control
        float spin_elapsed = 0.0f;

        // Reducer angle
        float last_angle = 0.0f;
        float reducer = 0.0f;
        bool initialized = false;

        // RPM calculation
        float   rpm_last_angle  = 0.0f;
        int64_t rpm_last_time   = 0;
        float   rpm_filtered    = 0.0f;
        

        static constexpr float RPM_LPF      = 0.20f; 
        static constexpr float RPM_UNWRAP_THR     = 180.0f;
        static constexpr int64_t RPM_MIN_DT_US    = 100;    // guard against /0 on first call
        
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
