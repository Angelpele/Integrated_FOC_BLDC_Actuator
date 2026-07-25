//===================================================================================================//
//                                              HALL Sensor                                          //
//                                                Encoder                                            //
//===================================================================================================//

#ifndef HALL_H
#define HALL_H
#include "esp32_S3.h"

class HALL_SENSOR {
    public:
        void init(int pp);
        float getHallAngle();
    private:
        uint8_t hallA = 0;
        uint8_t hallB = 0;
        uint8_t hallC = 0;
        uint8_t state = 0;
        uint8_t pole_pairs = 15;
        int8_t prev_sector = -1; 
        float accumulated_elec_angle = 0.0f; // Continuous electrical angle in degrees
        float mech_angle = 0.0f;
        int8_t state_to_sector[8] = {-1, 0, 2, 1, 4, 5, 3, -1};
};

#endif