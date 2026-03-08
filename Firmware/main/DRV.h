//================================================================================================================//
//                                                 DRV 8302 & Peripherals                                         //
//================================================================================================================//
#ifndef DRV_H
#define DRV_H 


#include "esp32_S3.h"
#include "stdio.h"
#include "driver/gpio.h"  //ESP32 GPIO
#include "driver/adc.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"
#include "driver/ledc.h"
#include "driver/mcpwm.h"
#include "esp_task_wdt.h"

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
                printf("FAULT OVERRIDE ENABLED\n");
            else
                printf("FAULT OVERRIDE DISABLED\n");
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

        // DRV current sensing
        float current_gain = 10.0; 

        // External sensors data
        int NTC_raw = 0;
        uint32_t last_ntc_read = 0;
        
        // Fault handling
        uint8_t latch_counter = 0;
        bool fault_override = false;

        // Waveform generation
        uint32_t PWM_freq = 20000; // 20 kHz carrier frequency
        float phase_accumulator = 0;
        float modulation_index = 1.0; // Amplitude (0.0 to 1.0)
        float electrical_freq = 0.5; // Motor Frequency (Hz)
        float dt = 0.0001; // Elapsed time (assuming 100us)

    };

class NTC_sensor {
    public:
        float getTemperature(int NTC_raw){
            return 1.0 / ((1.0f / T0) + (1.0f / B) * log((R0 / ((4095.0 / (float)NTC_raw) - 1.0)) / R0)) - 273.15 - offset;
        }
        void change_offset(int mod){
            offset+=(float)mod;
        }
    private:
        int B = 3380;               //Beta K of the NTC
        float R0 = 10000;           //Resistance at 25 degrees C
        float T0 = 25 + 273.15;     //Reference temperature in Kelvin
        float offset = 9.0;
};

#endif
