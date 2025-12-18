//===================================================================================================//
//                                          MT6701 Magnetometer                                      //
//                                                Encoder                                            //
//===================================================================================================//

#include "esp32.h"
#include "stdio.h"
#include "driver/gpio.h"  //ESP32 GPIO
#include "esp_rom_sys.h"

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

    float get_speed() const;
    float get_absolute_angle() const;
    int32_t get_revolutions() const;
    

private:    
    uint32_t SSI_read_raw();
    uint16_t SSI_get_ang(uint32_t raw);
    uint8_t SSI_get_mag_status(uint32_t raw);
    uint8_t SSI_get_CRC(uint32_t raw);
};

