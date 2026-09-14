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
        float accumulated_elec_angle = 0.0f;
        float mech_angle = 0.0f;
        int8_t state_to_sector[8] = {-1, 0, 2, 1, 4, 5, 3, -1};

        // --- Debounce state ---
        uint8_t pending_state   = 0;
        int64_t pending_since_us = 0;
        bool    have_pending    = false;

        // --- Interpolation state ---
        int64_t last_edge_time_us = 0;
        float   edge_period_us    = 0.0f;  // smoothed (EMA) period estimate
        int8_t  direction         = 0;
        bool    have_edge         = false;

        static constexpr float SECTOR_DEG        = 60.0f;
        static constexpr float MAX_COAST_PERIODS = 3.0f;
        static constexpr float PERIOD_EMA_ALPHA  = 0.3f;   // smoothing for edge_period_us
        static constexpr int64_t DEBOUNCE_US     = 150;    // min stable time before accepting a new state
        static constexpr float MIN_SANE_PERIOD_US = 300.0f; // fastest physically plausible single 60° step
};

#endif