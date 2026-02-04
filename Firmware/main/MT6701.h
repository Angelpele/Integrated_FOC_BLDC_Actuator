//===================================================================================================//
//                                          MT6701 Magnetometer                                      //
//                                                Encoder                                            //
//===================================================================================================//

#include "esp32_S3.h"
#include "stdio.h"
#include "driver/gpio.h"  //ESP32 GPIO
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define Delay_MAG 6   //Delay between CLK edges (30 ns min for MT6701)

// Estructura de datos original
typedef struct {
    float angle;
    uint8_t status;
    uint8_t push;
} mag_data;

class MT6701 {
public:

    void init();
    float calibration();
    mag_data get_data();

    float getAngleDegrees() const;
    float getAngleRadians() const;
    float getTurns() const;
    float getRPM() const;

    int32_t   getFullTurns() const;
    int32_t   getAccumulator() const;
    
    
private:    
    uint32_t SSI_read_raw();
    uint16_t SSI_get_ang(uint32_t raw);
    uint8_t SSI_get_mag_status(uint32_t raw);
    uint8_t SSI_get_CRC(uint32_t raw);

    int lastCount = 0;
    int accumulator = 0;

    float rpm = 0.0f;
    uint32_t lastUpdateTime = 0;

    static constexpr int   COUNTS_PER_REV = 16384;
    static constexpr float COUNTS_TO_DEG  = 360.0f / COUNTS_PER_REV;
    static constexpr float COUNTS_TO_RAD  = 6.28318530718f / COUNTS_PER_REV;

    SemaphoreHandle_t rpmMutex = nullptr;
    void updateCount();
};

