//===================================================================================================//
//                                          BMI270 IMU + Gyro                                        //
//                                            16 bit 3 axis                                          //
//===================================================================================================//
#ifndef BMI270_H
#define BMI270_H

#include "esp32_S3.h"
#include "stdio.h"
#include "driver/gpio.h"  //ESP32 GPIO
#include "esp_rom_sys.h"
#include "bmi2_defs.h"
#include "bmi270.h"
#include "bmi2.h"

typedef struct {
    float accx = 0, accy = 0, accz = 0;
    float gyrx = 0, gyry = 0, gyrz = 0;
    uint8_t ffall = 0;
} BMI_data;

enum BMI_Status {
    BMI_OK = 0,
    BMI_ERR = -1
};

class BMI270 {
public:
    BMI_Status init();
    BMI_Status calibration();
    BMI_data getdata();

    BMI_Status fault_detection();


    void set_override(bool state){
        fault_override = state;
        if (state)
            printf("BMI FAULT OVERRIDE ENABLED\n");
        else
            printf("BMI FAULT OVERRIDE DISABLED\n");
    }
    
    
private:    
    struct bmi2_dev dev; 
    bool fault_override = false;

    float accel_bias[3] = {0.0f, 0.0f, 0.0f};
    float gyro_bias[3]  = {0.0f, 0.0f, 0.0f};
    
    // Constants for LSB to Physical unit conversion (2G range and 2000dps range)
    const float ACCEL_RES_2G = 0.000061035f; // 1/16384 g/LSB
    const float GYRO_RES_2000 = 0.061035f;   // 1/16.384 dps/LSB
};

#endif
