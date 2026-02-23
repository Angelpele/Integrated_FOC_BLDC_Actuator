//===================================================================================================//
//                                              DRV 8302                                             //
//===================================================================================================//
#ifndef DRV8302_H
#define DRV8302_H 


#include "ESP32_S3.h"
#include "stdio.h"
#include "driver/gpio.h"  //ESP32 GPIO
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class DRV8302 {
    public:
        void init();
        void setup();
        void loop();
        float calibration();
    private:
};


#endif
