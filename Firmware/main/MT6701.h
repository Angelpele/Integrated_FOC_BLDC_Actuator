//===================================================================================================//
//                                          MT6701 Magnetometer                                      //
//                                                Encoder                                            //
//===================================================================================================//
#ifndef MT6701_H
#define MT6701_H

#include "esp32_S3.h"


#define Delay_MAG 6   //Delay between CLK edges (30 ns min for MT6701)

// Estructura de datos original
typedef struct {
    float angle;
    uint8_t status;
    uint8_t push;
} mag_data;

enum MT6701_Status {
    MT6701_OK = 0,
    MT6701_ERR_STRONG_FIELD = -1,
    MT6701_ERR_WEAK_FIELD = -2,
    MT6701_ERR_UNKNOWN = -99
};

class MT6701 {
public:
    void init();
    MT6701_Status calibration();
    mag_data getdata();
    void printdata(mag_data);

    MT6701_Status fault_detection();

    uint32_t getraw(){ return SSI_read_raw();}
    float getAngleDegrees() const;
    float getAngleRadians() const;
    float getTurns() const;
    float getRPM() const;

    int32_t getFullTurns() const;
    int32_t getAccumulator() const;

    void set_override(bool state){
        fault_override = state;
        if (state)
            printf("MT6701 FAULT OVERRIDE ENABLED\n");
        else
            printf("MT6701 FAULT OVERRIDE DISABLED\n");
    }
    
    
private:    
    uint32_t SSI_read_raw();
    uint16_t SSI_get_ang(uint32_t raw);
    uint8_t SSI_get_mag_status(uint32_t raw);
    uint8_t SSI_get_CRC(uint32_t raw);

    int lastCount = 0;
    int accumulator = 0;
    int offset = 0;

    float rpm = 0.0f;
    uint32_t lastUpdateTime = 0;
    bool fault_override = false;

    static constexpr int   COUNTS_PER_REV = 16384;
    static constexpr float COUNTS_TO_DEG  = 360.0f / COUNTS_PER_REV;
    static constexpr float COUNTS_TO_RAD  = 6.28318530718f / COUNTS_PER_REV;

    SemaphoreHandle_t rpmMutex = nullptr;
    void updateCount();
};

#endif

