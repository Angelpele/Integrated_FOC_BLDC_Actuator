//===================================================================================================//
//                                              HALL Sensor                                          //
//                                                Encoder                                            //
//===================================================================================================//

#include "hall.h"

void HALL_SENSOR::init(int pp)
{
    pole_pairs = pp;
    gpio_set_direction(HALL_A, GPIO_MODE_INPUT);
    gpio_set_direction(HALL_B, GPIO_MODE_INPUT);
    gpio_set_direction(HALL_C, GPIO_MODE_INPUT);
    gpio_set_pull_mode(HALL_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(HALL_B, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(HALL_C, GPIO_PULLUP_ONLY);
    
    hallA = gpio_get_level(HALL_A);
    hallB = gpio_get_level(HALL_B);
    hallC = gpio_get_level(HALL_C);
    state = (hallC << 2) | (hallB << 1) | hallA;

    // Set starting angle based on the initial state (electrical angle)
    int start_sector = state_to_sector[state];
    
    if (start_sector != -1) {
        accumulated_elec_angle = start_sector * 180.0f;
    } else {
        accumulated_elec_angle = 0.0f;
    }
}


float HALL_SENSOR::getHallAngle() {
    uint8_t hA = gpio_get_level(HALL_A);
    uint8_t hB = gpio_get_level(HALL_B);
    uint8_t hC = gpio_get_level(HALL_C);
    uint8_t new_state = (hC << 2) | (hB << 1) | hA;

    if (new_state != state) {
        int8_t prev_sector = state_to_sector[state];
        int8_t new_sector  = state_to_sector[new_state];

        if (prev_sector != -1 && new_sector != -1) {
            int8_t delta = new_sector - prev_sector;
            if (delta > 3)  delta -= 6;   // shortest path on a 6-sector circle
            if (delta < -3) delta += 6;
            accumulated_elec_angle += delta * 60.0f;
        }    
        state = new_state;
    }


    float max_elec_angle = 360.0f * pole_pairs;
    accumulated_elec_angle = fmodf(accumulated_elec_angle, max_elec_angle);
    if (accumulated_elec_angle < 0.0f) accumulated_elec_angle += max_elec_angle;

    mech_angle = accumulated_elec_angle / pole_pairs;
    return mech_angle;
}