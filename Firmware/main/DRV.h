//================================================================================================================//
//                                                 DRV 8302 & Peripherals                                         //
//================================================================================================================//
#ifndef DRV_H
#define DRV_H 


#include "esp32_S3.h"


enum DRV_Error {
    DRV_OK = 0,
    DRV_OVERVOLTAGE = 1,
    DRV_FAULT = 2,
    DRV_WARNING = 3,
    DRV_LATCHED = 4
};

typedef struct {
    float PVDD;
    float current1;
    float current2;
    uint16_t A_raw;
    uint16_t B_raw;
    uint16_t C_raw;
    uint16_t NFAULT_raw;
    uint16_t NOCTW_raw;
    float temperature;
}DRV_data;


class DRV {
    public:
        int init();
        int calibration();
        DRV_data getdata();
        void printdata(DRV_data);
        int fault_detection();

        void pwm_init();
        void setup_mcpwm();
        void update_spwm();
        void test_working_trans();
        void test_sinu();

        void set_override(bool state){
            fault_override = state;
            if (state)
                printf("DRV FAULT OVERRIDE ENABLED\n");
            else
                printf("DRV FAULT OVERRIDE DISABLED\n");
        }

        float getPVDD(int PVDD_raw){
            float R_frac = 7.8;  // 68k + 10k / 10k
            return (float)PVDD_raw * 3.3 / 4095 * R_frac;
        }
        void set_gain(float gain){
            current_gain = gain;
        }
        void set_MOC(bool state){
            gpio_set_level(MOC, state ? 1 : 0);
        }

        int get_RPM(float electrical_freq){
            return (int)(electrical_freq * 60 / MOTOR_POLE_PAIRS);
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
        float overvoltage = 18;
        uint16_t SO1_offset = 2048; // Default to mid-range (1.65V)
        uint16_t SO2_offset = 2048;

        // Motor parameters (5010 750KV)
        static constexpr int MOTOR_POLE_PAIRS = 7;   // 14 poles
        static constexpr int MOTOR_RESISTANCE = 270; // mOhm
        static constexpr int MOTOR_MAX_CURRENT = 40; // Amps

        // DRV current sensing
        float current_gain = 40.0; 

        // External sensors data
        int NTC_raw = 0;
        uint32_t last_ntc_read = 0;
        
        // Fault handling
        uint8_t latch_counter = 0;
        bool fault_override = false;

        // Waveform generation
        uint32_t PWM_freq = 20000; // 20 kHz carrier frequency
        float phase_accumulator = 0;
        float modulation_index = 1.0; // Amplitude (0.0 to 1.0) if higher -> overmodulation
        float electrical_freq = 2; // Motor Frequency (Hz), speed of the rotating field, higher than 0.5 to avoid deadzone
        float target_electrical_freq = 100; // Target frequency for smooth transitions
        float ramp_rate = 100; // How fast to change the frequency (Hz per second)
        bool ramp_done = false; 
        float dt = 0.0001; // Elapsed time (assuming 100us)

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
