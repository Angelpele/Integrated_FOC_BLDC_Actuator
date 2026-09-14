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

    int start_sector = state_to_sector[state];
    accumulated_elec_angle = (start_sector != -1) ? start_sector * 180.0f : 0.0f;

    int64_t now = esp_timer_get_time();
    last_edge_time_us = now;
    pending_since_us  = now;
    pending_state     = state;
    have_pending      = false;
    edge_period_us    = 0.0f;
    direction         = 0;
    have_edge         = false;
}

float HALL_SENSOR::getHallAngle() {
    uint8_t hA = gpio_get_level(HALL_A);
    uint8_t hB = gpio_get_level(HALL_B);
    uint8_t hC = gpio_get_level(HALL_C);
    uint8_t raw_state = (hC << 2) | (hB << 1) | hA;

    int64_t now_us = esp_timer_get_time();
    float max_elec_angle = 360.0f * pole_pairs;

    // ---- Software debounce: require the new reading to be stable for DEBOUNCE_US ----
    if (raw_state != state) {
        if (!have_pending || pending_state != raw_state) {
            pending_state    = raw_state;
            pending_since_us = now_us;
            have_pending     = true;
        } else if (now_us - pending_since_us >= DEBOUNCE_US) {
            // Accepted as a real transition
            uint8_t new_state = raw_state;
            int8_t prev_sector = state_to_sector[state];
            int8_t new_sector  = state_to_sector[new_state];

            if (prev_sector != -1 && new_sector != -1) {
                int8_t delta = new_sector - prev_sector;
                if (delta > 3)  delta -= 6;
                if (delta < -3) delta += 6;

                accumulated_elec_angle += delta * SECTOR_DEG;

                if (delta == 1 || delta == -1) {
                    float period = (float)(now_us - last_edge_time_us);
                    if (period > MIN_SANE_PERIOD_US && period < 2000000.0f) {
                        // Smooth instead of snapping to a single sample
                        if (edge_period_us <= 0.0f) edge_period_us = period;
                        else edge_period_us += PERIOD_EMA_ALPHA * (period - edge_period_us);
                        direction = (delta > 0) ? 1 : -1;
                        have_edge = true;
                    }
                    // else: implausibly fast single-step -> keep old edge_period_us, ignore timing
                } else if (delta != 0) {
                    direction = (delta > 0) ? 1 : -1;
                }
            }
            state = new_state;
            last_edge_time_us = now_us;
            have_pending = false;
        }
    } else {
        have_pending = false; // reading settled back to the original state -> was noise
    }

    accumulated_elec_angle = fmodf(accumulated_elec_angle, max_elec_angle);
    if (accumulated_elec_angle < 0.0f) accumulated_elec_angle += max_elec_angle;

    float interpolated_angle = accumulated_elec_angle;

    if (have_edge && edge_period_us > 0.0f) {
        float dt_since_us = (float)(now_us - last_edge_time_us);
        if (dt_since_us < edge_period_us * MAX_COAST_PERIODS) {
            float frac = dt_since_us / edge_period_us;
            if (frac > 1.0f) frac = 1.0f;
            interpolated_angle += direction * frac * SECTOR_DEG;
        }
    }

    interpolated_angle = fmodf(interpolated_angle, max_elec_angle);
    if (interpolated_angle < 0.0f) interpolated_angle += max_elec_angle;

    mech_angle = interpolated_angle / pole_pairs;
    return mech_angle;
}