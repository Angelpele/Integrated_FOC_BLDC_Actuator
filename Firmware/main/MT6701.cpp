//===================================================================================================//
//                                          MT6701 Magnetometer                                      //
//                                                Encoder                                            //
//===================================================================================================//

#include "MT6701.h"
#include "esp_timer.h"


void MT6701::init()
{
    gpio_set_direction(MAG_DO, GPIO_MODE_INPUT);
    gpio_set_direction(MAG_CLK, GPIO_MODE_OUTPUT);
    gpio_set_direction(MAG_CSN, GPIO_MODE_OUTPUT);

    gpio_set_level(MAG_CLK, 0);
    gpio_set_level(MAG_CSN, 1);     // Active LOW
    //Power up time Max 32 ms
}

MT6701_Status MT6701::calibration()
{                                  // Return status code
    uint32_t sum = 0;
    uint32_t data = 0;
    int8_t error = 0;
    gpio_set_level(MAG_CSN, 0);
    esp_rom_delay_us(1);
    printf("Calibrating Magnetic Encoder...");
    for(int i=0; i<1000;i++)
    {
        data = SSI_read_raw();
        sum += SSI_get_ang(data);
        uint8_t mag = SSI_get_mag_status(data);
        if(mag & 0x1) error++;          // Too Strong field, bit 0
        if(mag & 0x2) error--;          // Too Weak field, bit 1
        esp_rom_delay_us(2000/(2*Delay_MAG));
    }
    if(abs(error) > 300)
    {
        printf("Magnetic Encoder Calibration error: %d\n", error);
        if (error > 0) return MT6701_ERR_STRONG_FIELD;
        else return MT6701_ERR_WEAK_FIELD;
    }
    esp_rom_delay_us(Delay_MAG);
    gpio_set_level(MAG_CSN, 1);
    offset = ((float)sum / 1000.0f) / COUNTS_PER_REV * 360.0f;
    return MT6701_OK;
}

MT6701_Status MT6701::fault_detection()
{
    if (fault_override){
        return MT6701_OK;   // Ignore ALL faults
    }

    uint32_t data = SSI_read_raw();
    uint8_t mag = SSI_get_mag_status(data);
    // Bit 0: Too strong, Bit 1: Too weak, Bit 3: Loss of track
    if (mag & 0x8) {
        printf("MT6701 FAULT: Magnetic field too strong\n");
        return MT6701_ERR_STRONG_FIELD;
    }
    if (mag & 0x4) {
        printf("MT6701 FAULT: Magnetic field too weak\n");
        return MT6701_ERR_WEAK_FIELD;
    }
    if (mag & 0x2) {
        printf("MT6701 WARNING: Loss of track\n");
        return MT6701_ERR_UNKNOWN;
    }
    return MT6701_OK;
}

uint32_t MT6701::SSI_read_raw()
{
    uint32_t raw = 0;
    gpio_set_level(MAG_CSN, 0);
    
    for (int i = 0; i < 24; i++)
    {
        gpio_set_level(MAG_CLK, 1);             //MAG_DO reads the value on the rising edge of CLK
        __asm__ __volatile__("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");  //Required delay 30 ns
        raw <<= 1;                              // Shift MSB received to the left 1 bit
        raw |= gpio_get_level(MAG_DO);          // Set value of MAG_DO in LSB
        gpio_set_level(MAG_CLK, 0);
        __asm__ __volatile__("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
    }
    gpio_set_level(MAG_CSN, 1); 
    return raw;
}

mag_data MT6701::getdata()
{
    uint32_t data = 0;
    mag_data package;

    data = SSI_read_raw();
    float angle = ((float)SSI_get_ang(data) / COUNTS_PER_REV * 360.0f) - offset;
    angle = fmodf(angle, 360.0f);   // Wrap angle to [0, 360)
    if (angle < 0) angle += 360.0f;
    package.angle = angle;

    uint8_t mg = SSI_get_mag_status(data);
    package.push = (mg >> 0) & 0x01;             // Active HIGH, shift 2 bits to the right and mask
    package.status = (mg >> 1) & 0x01;           // If 1, loss of track
    
    return package;
}

uint16_t MT6701::SSI_get_ang (uint32_t raw)
{
    return (uint16_t)((raw >> 10) & 0x3FFF);   // Angle bits from bit 0:13 MSB first D[13:0], masked with 14 "0" bits
}

uint8_t MT6701::SSI_get_mag_status (uint32_t raw)
{
    return (uint8_t)((raw >> 6) & 0x0F);   // Mag Status bits from bit 14:17 MSB first Mg[3:0], masked with 4 "0" bits
}

uint8_t MT6701::SSI_get_CRC (uint32_t raw)
{
    return (uint8_t)(raw & 0x3F);   // Cyclic Redundancy Check from bit 18:23 MSB first CRC[5:0], masked with 6 "0" bits
}
