//===================================================================================================//
//                                          MT6701 Magnetometer                                      //
//                                                Encoder                                            //
//===================================================================================================//

#include "MT6701.h"



void MT6701::init()
{
    gpio_set_direction(MAG_DO, GPIO_MODE_INPUT);
    gpio_set_direction(MAG_CLK, GPIO_MODE_OUTPUT);
    gpio_set_direction(MAG_CSN, GPIO_MODE_OUTPUT);

    gpio_set_level(MAG_CLK, 0);
    gpio_set_level(MAG_CSN, 1);     // Active LOW
    //Power up time Max 32 ms
}

float MT6701::calibration()
{                                  // Return offset value
    uint32_t sum = 0;
    uint32_t data = 0;
    int8_t error = 0;
    gpio_set_level(MAG_CSN, 0);
    esp_rom_delay_us(1);
    
    for(int i=0; i<1000;i++)
    {
        data = SSI_read_raw();
        sum += SSI_get_ang(data);
        
        uint8_t mag = SSI_get_mag_status(data);
        if(mag & 0x1) error++;          // Too Strong field, bit 0
        if(mag & 0x2) error--;          // Too Weak field, bit 1
        
        esp_rom_delay_us(2000/(2*Delay_MAG));
    }
    
    if(abs(error) > 500)
    {
        return error > 0 ? -1.0f : -2.0f;                   //-1 for Strong field error, -2 for weak field error
    }

    esp_rom_delay_us(Delay_MAG);
    gpio_set_level(MAG_CSN, 1);
    return ((float)sum/16384000 * 360);
}

uint32_t MT6701::SSI_read_raw()
{
    uint32_t raw = 0;
    
    for (int i = 0; i < 24; i++)
    {
        gpio_set_level(MAG_CLK, 1);             //MAG_DO reads the value on the rising edge of CLK
        esp_rom_delay_us(Delay_MAG);
        raw <<= 1;                              // Shift MSB received to the left 1 bit
        raw |= gpio_get_level(MAG_DO);          // Set value of MAG_DO in LSB
        gpio_set_level(MAG_CLK, 0);
        esp_rom_delay_us(Delay_MAG);
    }
    return raw;
}

mag_data MT6701::get_data()
{
    uint32_t sum = 0;
    uint32_t data = 0;
    mag_data package;

    data = SSI_read_raw();
    package.angle = (float)SSI_get_ang(data)/16384 * 360;
    uint8_t mg = SSI_get_mag_status(data);
    package.push = (mg >> 2) & 0x01;             // Active HIGH, shift 2 bits to the right and mask
    package.status = (mg >> 3) & 0x01;           // If 1, loss of track
    
    return package;
}

uint16_t MT6701::SSI_get_ang (uint32_t raw)
{
    return (uint16_t)(raw & ((1U << 14) - 1U));   // Angle bits from bit 0:13 MSB first D[13:0], masked with 14 "0" bits
}

uint8_t MT6701::SSI_get_mag_status (uint32_t raw)
{
    return (uint8_t)((raw >> 14) & ((1U << 4) - 1U));   // Mag Status bits from bit 14:17 MSB first Mg[3:0], masked with 4 "0" bits
}

uint8_t MT6701::SSI_get_CRC (uint32_t raw)
{
    return (uint8_t)((raw >> 18) & ((1U << 6) - 1U));   // Cyclic Redundancy Check from bit 18:23 MSB first CRC[5:0], masked with 6 "0" bits
}

