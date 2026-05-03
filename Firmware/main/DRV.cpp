//===================================================================================================//
//                                              DRV 8302                                             //
//===================================================================================================//
#include "DRV.h"

NTC_sensor ntc;

int DRV::init()
{
    esp_rom_gpio_pad_select_gpio(DC_CAL);
    esp_rom_gpio_pad_select_gpio(MPWM);
    esp_rom_gpio_pad_select_gpio(MOC);
    esp_rom_gpio_pad_select_gpio(NFAULT);

    gpio_reset_pin(DC_CAL);
    gpio_reset_pin(MPWM);
    gpio_reset_pin(MOC);
    gpio_reset_pin(NFAULT);

    gpio_set_direction(DC_CAL, GPIO_MODE_OUTPUT);
    gpio_set_direction(MPWM, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOC, GPIO_MODE_OUTPUT);

    gpio_set_level(INL_A, 0);
    gpio_set_level(INH_A, 0);
    gpio_set_level(INL_B, 0);
    gpio_set_level(INH_B, 0);
    gpio_set_level(INL_C, 0);
    gpio_set_level(INH_C, 0);

    adc1_config_width(ADC_WIDTH_BIT_12);  // No need to configure ADC2
    adc1_config_channel_atten(A_SENSE, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(B_SENSE, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(SO1, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(SO2, ADC_ATTEN_DB_12);
    adc2_config_channel_atten(PVDD_SENSE, ADC_ATTEN_DB_12);
    adc2_config_channel_atten(NTC, ADC_ATTEN_DB_12);

    gpio_set_direction(NFAULT, GPIO_MODE_INPUT);
    gpio_set_direction(NOCTW, GPIO_MODE_INPUT);
   
    gpio_set_direction(GAIN, GPIO_MODE_OUTPUT);

    gpio_set_direction(EN_DRV, GPIO_MODE_OUTPUT); // Set to output to drive the pin
    gpio_set_level(EN_DRV, 0);        // keep driver disabled first
    gpio_set_level(DC_CAL, 0);        // normal operation
    gpio_set_level(GAIN, 1);          // choose gain explicitly
    gpio_set_level(MPWM, 0);          // 6 PWM indep inputs
    gpio_set_level(MOC, 0);           // 0: cycle by cycle current control
    for (int i = 0; i < 50; i++) {      // 50 × 100ms = 5000ms
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_task_wdt_reset();
    }
    gpio_set_level(EN_DRV, 1);
    for (int i = 0; i < 10; i++) {      // 10 × 100ms = 1000ms
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_task_wdt_reset();
    }
    NFAULT_raw = gpio_get_level(NFAULT);

    if (NFAULT_raw == 0) {
        printf("ERROR: DRV8302 FAULT ERROR\n");
        return 1;
    } else {
        printf("DRV8302 POWERED UP\n");
    }
    return 0;
}

int DRV::calibration() 
{
    gpio_set_level(DC_CAL, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    long sum1 = 0;
    long sum2 = 0;
    int samples = 500;

    for(int i=0; i < samples; i++){
        sum1 += adc1_get_raw(SO1);
        sum2 += adc1_get_raw(SO2);
    }

    SO1_offset = sum1/samples;
    SO2_offset = sum2/samples;

    gpio_set_level(DC_CAL, 0);

    //Restore C_sense pin to normal operation
    adc1_config_channel_atten(C_SENSE, ADC_ATTEN_DB_12);

    printf("DRV Offsets -> SO1: %d | SO2: %d\n", SO1_offset, SO2_offset);

    // Basic sanity check: Offset should be roughly middle of 12-bit range (2048)
    if (SO1_offset < 1000 || SO1_offset > 3000) {
        printf("WARNING: SO1 Offset unusual. Check hardware!\n");
        return -1; 
    }
    printf("DRV CALIBRATION COMPLETE\n");
    return 0; // Return 0 on success
}

DRV_data DRV::getdata() {
    DRV_data package;

    // ---- Digital inputs ----
    package.NFAULT_raw = gpio_get_level(NFAULT);
    package.NOCTW_raw = gpio_get_level(NOCTW);
    
    // ---- ADC1 ----
    package.A = adc1_get_raw(A_SENSE)*voltage_gain;
    package.B = adc1_get_raw(B_SENSE)*voltage_gain;
    package.C = adc1_get_raw(C_SENSE)*voltage_gain;
    SO1_raw = adc1_get_raw(SO1);
    SO2_raw = adc1_get_raw(SO2);
    //SO1_raw = 0;
    //SO2_raw = 0;
    package.current1 = (float)(SO1_offset - SO1_raw) * voltage_step / (current_gain * R_shunt);
    package.current2 = (float)(SO2_offset - SO2_raw) * voltage_step / (current_gain * R_shunt);
    
    // ---- ADC2 ----
    adc2_get_raw(PVDD_SENSE, ADC_WIDTH_BIT_12, &PVDD_raw);
    adc2_get_raw(NTC, ADC_WIDTH_BIT_12, &NTC_raw);
    package.PVDD = getPVDD(PVDD_raw);


    if (esp_log_timestamp() - last_ntc_read > 500) {
        last_temperature = ntc.getTemperature(NTC_raw); 
        last_ntc_read = esp_log_timestamp();
    }
    package.temperature = last_temperature;
    return package;
}

int DRV::fault_detection() {
    if (fault_override) return DRV_OK;

    int error_num = DRV_OK;

    if (getPVDD(PVDD_raw) >= overvoltage) {
        gpio_set_level(EN_DRV, 0);
        printf("PVDD VOLTAGE OVER LIMIT!\n");
        error_num = DRV_OVERVOLTAGE;
    }
    else if (NFAULT_raw == 0 && NOCTW_raw == 0) {
        printf("LATCHED FAULT detected!\n");
        latch_counter++;
        error_num = DRV_LATCHED;
    }
    else if (NFAULT_raw == 0) {
        nfault_glitch_count++;
        if (nfault_glitch_count >= 4) {
            printf("FAULT detected in DRV8302!\n");
            latch_counter++;
            nfault_glitch_count = 0;
        }
        error_num = DRV_FAULT;
    }
    else if (NOCTW_raw == 0) {
        printf("Overtemperature / Overcurrent Warning!\n");
        latch_counter++;
        error_num = DRV_WARNING;
    }
    else {
        nfault_glitch_count = 0;
        if (latch_counter > 0)
            latch_counter = 0;
    }

    if (latch_counter >= 1000) {
        printf("Fault threshold reached -> Driver must be reset\n");
        return DRV_LATCHED;
    }

    return error_num;
}

#define GPIO_PWM_UNIT MCPWM_UNIT_0

void DRV::setup_mcpwm() {
    // Initialize pins for 3 phases (6 total signals)
    mcpwm_gpio_init(GPIO_PWM_UNIT, MCPWM0A, INH_A); 
    mcpwm_gpio_init(GPIO_PWM_UNIT, MCPWM0B, INL_A);
    mcpwm_gpio_init(GPIO_PWM_UNIT, MCPWM1A, INH_B);
    mcpwm_gpio_init(GPIO_PWM_UNIT, MCPWM1B, INL_B);
    mcpwm_gpio_init(GPIO_PWM_UNIT, MCPWM2A, INH_C);
    mcpwm_gpio_init(GPIO_PWM_UNIT, MCPWM2B, INL_C);

    mcpwm_config_t pwm_config = {
        .frequency = PWM_freq,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,       // Duty as active high 
        .counter_mode = MCPWM_UP_DOWN_COUNTER // Center-aligned PWM for fewer harmonics
    };

    mcpwm_init(GPIO_PWM_UNIT, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(GPIO_PWM_UNIT, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(GPIO_PWM_UNIT, MCPWM_TIMER_2, &pwm_config);

    // Configure Dead-time
    // 1us software dead-time for safety
    mcpwm_deadtime_enable(GPIO_PWM_UNIT, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 40, 40);
    mcpwm_deadtime_enable(GPIO_PWM_UNIT, MCPWM_TIMER_1, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 40, 40);
    mcpwm_deadtime_enable(GPIO_PWM_UNIT, MCPWM_TIMER_2, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 40, 40);
}

AlignOffset DRV::align(float angle) {
    AlignOffset result = {false, 0.0f};

    float duty = (align_time < ramp_duration) ? (align_time / ramp_duration) * ALIGN_DUTY : ALIGN_DUTY;

    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_B, 100.0f - duty);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_A, 0.0f);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_B, 100.0f);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_A, 0.0f);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_B, 100.0f);
    align_time += dt;

    if (align_time >= ALIGN_DURATION) {

        float theta_mag = angle * (M_PI / 180.0f);
        elec_offset = -theta_mag * (float)MOTOR_POLE_PAIRS;

        while (elec_offset >  M_PI) elec_offset -= 2.0f * M_PI;
        while (elec_offset < -M_PI) elec_offset += 2.0f * M_PI;

        phase_accumulator = 0.0f;
        align_time = 0.0f; 

        printf("Alignment done. Angle: %.2f deg | elec_offset: %.4f rad\n", angle, elec_offset);

        result.done = true;
        result.elec_offset = elec_offset;
    }
    esp_task_wdt_reset();
    return result;
}

// ============= Basic actions ================

bool DRV::ramp(float angle, float init_freq, float obj_freq) {
    
    if (!ramp_active) {                   //Initialize ramp -> first call
        electrical_freq = init_freq;
        ramp_target_freq = obj_freq;
        ramp_active = true;
    }

    float step = (fabsf(obj_freq - init_freq) / ramp_duration) * dt;
    if (ramp_target_freq > electrical_freq){
        electrical_freq = fminf(electrical_freq + step, ramp_target_freq);
    }else{
        electrical_freq = fmaxf(electrical_freq - step, obj_freq);
    }

    phase_accumulator += 2.0f * M_PI * electrical_freq * dt;
    if (phase_accumulator >= 2.0f * M_PI) phase_accumulator -= 2.0f * M_PI;
    if (phase_accumulator <  0.0f) phase_accumulator += 2.0f * M_PI;

    float theta_mag = angle * (M_PI / 180.0f);
    float theta_elec_raw = fmodf(theta_mag * (float)MOTOR_POLE_PAIRS + elec_offset, 2.0f * M_PI);
    if (theta_elec_raw < 0.0f) theta_elec_raw += 2.0f * M_PI;

    float blend    = fabsf(electrical_freq - init_freq) / fabsf(obj_freq - init_freq + 1e-6f);
    float theta_elec = (1.0f - blend) * phase_accumulator + blend * theta_elec_raw;

    float duty_a = (sinf(theta_elec)                       * modulation_index + 1.0f) * 50.0f;
    float duty_b = (sinf(theta_elec - 2.0f * M_PI / 3.0f)  * modulation_index + 1.0f) * 50.0f;
    float duty_c = (sinf(theta_elec - 4.0f * M_PI / 3.0f)  * modulation_index + 1.0f) * 50.0f;

    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_A, duty_a);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_B, 100.0f - duty_a);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_A, duty_b);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_B, 100.0f - duty_b);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_A, duty_c);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_B, 100.0f - duty_c);

    bool done = (fabsf(electrical_freq - obj_freq) < 0.1f);
    if (done)                   // Reset ramp state
    {
        ramp_active = false;
        electrical_freq = obj_freq;
        phase_accumulator = theta_elec_raw; 
    }
    return done;
}

void DRV::brake() {
    // Dynamic braking
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0f);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_B, 100.0f);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_A, 0.0f);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_B, 100.0f);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_A, 0.0f);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_B, 100.0f);
}

void DRV::idle() {
    // Free rotor
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0f);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_B, 0.0f);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_A, 0.0f);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_B, 0.0f);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_A, 0.0f);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_B, 0.0f);
}

// ============= Spin functions ================

bool DRV::closed_spin(float angle, float freq) {

    if (spin_duration > 0.0f) {
        spin_elapsed += dt;
        if (spin_elapsed >= spin_duration) {
            spin_elapsed = 0.0f;
            theta_reset = true;
            return true;
        }
    }

    electrical_freq = freq;
    float theta_mag = angle * (M_PI / 180.0f);
    float theta_elec_raw = fmodf(theta_mag * (float)MOTOR_POLE_PAIRS + elec_offset, 2.0f * M_PI);
    if (theta_elec_raw < 0.0f) theta_elec_raw += 2.0f * M_PI;

    // Low pass filter for angle estimation
    static float theta_filtered = 0.0f;
    if (theta_reset) {
        theta_filtered = theta_elec_raw;  // Reset theta after changing mode and moving motor
        theta_reset = false;
    }
    theta_filtered = (1.0f - CLOSED_LOOP_LPF) * theta_filtered + CLOSED_LOOP_LPF * theta_elec_raw;

    float duty_a = (sinf(theta_filtered)                      * modulation_index + 1.0f) * 50.0f;
    float duty_b = (sinf(theta_filtered - 2.0f * M_PI / 3.0f) * modulation_index + 1.0f) * 50.0f;
    float duty_c = (sinf(theta_filtered - 4.0f * M_PI / 3.0f) * modulation_index + 1.0f) * 50.0f;

    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_A, duty_a);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_B, 100.0f - duty_a);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_A, duty_b);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_B, 100.0f - duty_b);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_A, duty_c);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_B, 100.0f - duty_c);
    return false;
}

float DRV::get_currents_angle() {
    SO1_raw = adc1_get_raw(SO1);
    SO2_raw = adc1_get_raw(SO2);
    float current1 = (float)(SO1_offset - SO1_raw) * voltage_step / (current_gain * R_shunt);
    float current2 = (float)(SO2_offset - SO2_raw) * voltage_step / (current_gain * R_shunt);

    static constexpr float L    = 0.000065f;    // ~65 uH estimated for 5010 750KV
    static constexpr float dt   = TIMER_INTERVAL_US * 0.000001f * FBK_DIVIDER;    // Fbk loop dt

    /*int PVDD_raw = 0;
    adc2_get_raw(PVDD_SENSE, ADC_WIDTH_BIT_12, &PVDD_raw);
    float Vbus = getPVDD(PVDD_raw);*/

    // Reconstruct third phase (Kirchhoff: ia + ib + ic = 0)
    float ia = current1;
    float ib = current2;
    float ic = -(ia + ib);

    // Clarke transform (3-phase → αβ)
    float i_alpha = ia;
    float i_beta  = (ia + 2.0f * ib) * (1.0f / 1.7320508f); // 1/sqrt(3)

    // Estimated voltages (open loop — assumes balanced PWM, no Park feedback yet)
    // Vbus/3 is the neutral point in balanced 3-phase
    static float v_alpha = 0.0f;
    static float v_beta  = 0.0f;

    // Flux observer state
    static float flux_alpha = 0.0f;
    static float flux_beta  = 0.0f;
    static float i_alpha_prev = 0.0f;
    static float i_beta_prev  = 0.0f;

    // Back-EMF = V - R*I - L*(dI/dt)
    float di_alpha = (i_alpha - i_alpha_prev) / dt;
    float di_beta  = (i_beta  - i_beta_prev)  / dt;

    float bemf_alpha = v_alpha - MOTOR_RESISTANCE * i_alpha - L * di_alpha;
    float bemf_beta  = v_beta  - MOTOR_RESISTANCE * i_beta  - L * di_beta;

    // Integrate back-EMF to get flux
    flux_alpha += bemf_alpha * dt;
    flux_beta  += bemf_beta  * dt;

    // Angle from flux
    float angle = atan2f(flux_beta, flux_alpha) * (180.0f / M_PI);
    if (angle < 0.0f) angle += 360.0f;

    // Update previous values
    i_alpha_prev = i_alpha;
    i_beta_prev  = i_beta;

    return angle; // deg 0 to 360
}

bool DRV::svpwm(float angle_deg, float freq) {

    if (!spin_timer_started) {
        spin_start_time = (float)esp_timer_get_time() / 1e6f;
        spin_timer_started = true;
    }
    float elapsed = (float)esp_timer_get_time() / 1e6f - spin_start_time;
    if (spin_duration > 0.0f && elapsed >= spin_duration) {
        spin_timer_started = false;   // reset for next call
        return true;                  // signal MOTION_IDLE
    }

    // ── Electrical angle in radians ───────────────────────
    float theta = angle_deg * (M_PI / 180.0f);

    // Normalize to [0, 2π)
    while (theta <  0.0f)        theta += 2.0f * M_PI;
    while (theta >= 2.0f * M_PI) theta -= 2.0f * M_PI;

    // ── Sector (0–5) ──────────────────────────────────────
    // Each sector = 60°
    int sector = (int)(theta / (M_PI / 3.0f));           // 0..5
    float theta_s = theta - sector * (M_PI / 3.0f);      // angle within sector [0, π/3)

    // ── SVM time fractions ────────────────────────────────
    // T1, T2: active vector times (normalized to 1 period)
    float T1 = modulation_index * sinf((M_PI / 3.0f) - theta_s);
    float T2 = modulation_index * sinf(theta_s);
    float T0 = 1.0f - T1 - T2;                           // zero-vector time

    // Clamp to avoid overmodulation
    if (T0 < 0.0f) {
        float scale = 1.0f / (T1 + T2);
        T1 *= scale; T2 *= scale; T0 = 0.0f;
    }

    // ── Duty cycles per sector ────────────────────────────
    // Center-aligned: split T0/2 at both ends
    float Ta, Tb, Tc;
    float half0 = T0 / 2.0f;

    switch (sector) {
        case 0: Ta = T1 + T2 + half0; Tb = T2 + half0; Tc = half0;          break;
        case 1: Ta = T1 + half0;      Tb = T1 + T2 + half0; Tc = half0;     break;
        case 2: Ta = half0;           Tb = T1 + T2 + half0; Tc = T2 + half0; break;
        case 3: Ta = half0;           Tb = T1 + half0;      Tc = T1 + T2 + half0; break;
        case 4: Ta = T2 + half0;      Tb = half0;           Tc = T1 + T2 + half0; break;
        case 5: Ta = T1 + T2 + half0; Tb = half0;           Tc = T1 + half0; break;
        default: Ta = Tb = Tc = 0.5f; break;
    }

    // ── Clamp duties to [0, 1] ────────────────────────────
    Ta = fmaxf(0.0f, fminf(1.0f, Ta));
    Tb = fmaxf(0.0f, fminf(1.0f, Tb));
    Tc = fmaxf(0.0f, fminf(1.0f, Tc));

    // ── Output to MCPWM ──────────────────────────────────
    // MCPWM_DUTY_MODE_0 + ACTIVE_HIGH_COMPLIMENT: A is high-side, B is complementary low-side
    // Dead-time is handled by hardware (40 ticks each side)
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_A, Ta * 100.0f);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_A, Tb * 100.0f);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_A, Tc * 100.0f);

    mcpwm_set_duty_type(GPIO_PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(GPIO_PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(GPIO_PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);

    return false;
}

bool DRV::run_openloop_bemf(float target_freq, float duty) {
    static int8_t prev_sign_A = 0, prev_sign_B = 0, prev_sign_C = 0;
    static uint8_t commutation_step = 0;
    static int64_t last_zc_time = 0;
    static float   commutation_period_us = 0.0f;

    // Reset static state when restarting
    if (!spin_timer_started) {
        prev_sign_A = prev_sign_B = prev_sign_C = 0;
        commutation_step = 0;
        last_zc_time = 0;
        commutation_period_us = 0.0f;
    }

    float elapsed = (float)esp_timer_get_time() / 1e6f - spin_start_time;
    if (spin_duration > 0.0f && elapsed >= spin_duration) {
        spin_timer_started = false;   // reset for next call
        return true;                  // signal MOTION_IDLE
    }

    // ── Read phase voltages ───────────────────────────────
    static constexpr float V_SCALE  = (3.3f / 4095.0f) * 11.0f;  // same ratio as PVDD
    static constexpr float ZC_HYSTERESIS = 0.2f;  // Volts — tune to reduce noise triggers

    float vA = adc1_get_raw(A_SENSE) * V_SCALE;
    float vB = adc1_get_raw(B_SENSE) * V_SCALE;
    float vC = adc1_get_raw(C_SENSE) * V_SCALE;

    // Virtual neutral point (average of all 3 phases)
    float vN = (vA + vB + vC) / 3.0f;

    // BEMF relative to neutral
    float bemf_A = vA - vN;
    float bemf_B = vB - vN;
    float bemf_C = vC - vN;

    // ── Zero-crossing detection ───────────────────────────
    // Track sign changes per phase

    auto sign_of = [&](float v) -> int8_t {
        if (v >  ZC_HYSTERESIS) return  1;
        if (v < -ZC_HYSTERESIS) return -1;
        return 0;
    };

    int8_t sA = sign_of(bemf_A);
    int8_t sB = sign_of(bemf_B);
    int8_t sC = sign_of(bemf_C);

    bool zero_crossed = false;

    // Detect zero crossing on the floating phase based on current step
    // Step defines which phase is floating (not driven)
    switch (commutation_step) {
        case 0: zero_crossed = (prev_sign_C == -1 && sC ==  1); break; // C rising
        case 1: zero_crossed = (prev_sign_B ==  1 && sB == -1); break; // B falling
        case 2: zero_crossed = (prev_sign_A == -1 && sA ==  1); break; // A rising
        case 3: zero_crossed = (prev_sign_C ==  1 && sC == -1); break; // C falling
        case 4: zero_crossed = (prev_sign_B == -1 && sB ==  1); break; // B rising
        case 5: zero_crossed = (prev_sign_A ==  1 && sA == -1); break; // A falling
    }

    if (sA != 0) prev_sign_A = sA;
    if (sB != 0) prev_sign_B = sB;
    if (sC != 0) prev_sign_C = sC;

    // ── Commutation timing ────────────────────────────────
    int64_t now_us = esp_timer_get_time();

    if (zero_crossed) {
        int64_t period = now_us - last_zc_time;
        if (period > 500 && period < 500000) {  // sanity: 2Hz–2kHz
            commutation_period_us = (float)period;
            electrical_freq = 1000000.0f / (6.0f * commutation_period_us); // Hz
        }
        last_zc_time = now_us;

        // Advance step after 30° delay (half the 60° step)
        // Delay = commutation_period_us / 2
        vTaskDelay(pdMS_TO_TICKS((uint32_t)(commutation_period_us / 2000.0f)));

        commutation_step = (commutation_step + 1) % 6;
    }

    // ── Open-loop fallback at low speed ──────────────────
    static constexpr float BEMF_MIN_FREQ = 3.0f;  // Hz — below this, ignore BEMF

    if (electrical_freq < BEMF_MIN_FREQ || commutation_period_us == 0.0f) {
        // Pure open loop — ramp up
        phase_accumulator += 2.0f * M_PI * electrical_freq * dt;
        if (phase_accumulator >= 2.0f * M_PI)
            phase_accumulator -= 2.0f * M_PI;

        if (electrical_freq < target_freq)
            electrical_freq += ramp_rate * dt;

        modulation_index = duty;
        return svpwm(phase_accumulator * (180.0f / M_PI), electrical_freq);
    }

    // ── 6-step commutation table ──────────────────────────
    // {INH_A, INL_A, INH_B, INL_B, INH_C, INL_C}
    // 1 = active high, -1 = active low, 0 = floating
    static constexpr int8_t step_table[6][3] = {
        { 1, -1,  0},   // Step 0: A high, B low,  C float
        { 1,  0, -1},   // Step 1: A high, C low,  B float
        { 0,  1, -1},   // Step 2: B high, C low,  A float
        {-1,  1,  0},   // Step 3: B high, A low,  C float
        {-1,  0,  1},   // Step 4: C high, A low,  B float
        { 0, -1,  1},   // Step 5: C high, B low,  A float
    };

    int8_t phA = step_table[commutation_step][0];
    int8_t phB = step_table[commutation_step][1];
    int8_t phC = step_table[commutation_step][2];

    // Convert step to PWM output
    auto set_phase = [&](gpio_num_t inh, gpio_num_t inl, int8_t state, float d) {
        float dc = d * modulation_index;
        if (state == 1) {           // High side PWM
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, dc * 100.0f);
            gpio_set_level(inl, 0);
        } else if (state == -1) {   // Low side ON
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
            gpio_set_level(inl, 1);
        } else {                    // Float
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
            gpio_set_level(inl, 0);
        }
    };

    modulation_index = duty;
    set_phase(INH_A, INL_A, phA, duty);
    set_phase(INH_B, INL_B, phB, duty);
    set_phase(INH_C, INL_C, phC, duty);

    return false;
}

// ============= Test functions ================

void DRV::test_working_trans() {
    struct TestStep {
        const char* msg;
        int high_pin;
        int low_pin;
    };

    TestStep steps[] = {
        {"FASE A (+) | FASE B (-)", INH_A, INL_B},
        {"FASE A (-) | FASE B (+)", INH_B, INL_A},
        {"FASE B (+) | FASE C (-)", INH_B, INL_C},
        {"FASE B (-) | FASE C (+)", INH_C, INL_B},
        {"FASE C (+) | FASE A (-)", INH_C, INL_A},
        {"FASE C (-) | FASE A (+)", INH_A, INL_C}
    };

    for (int i = 0; i < 6; i++) {        
        gpio_set_level((gpio_num_t)steps[i].high_pin, 1);
        gpio_set_level((gpio_num_t)steps[i].low_pin, 1);
        
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level((gpio_num_t)steps[i].high_pin, 0);
        gpio_set_level((gpio_num_t)steps[i].low_pin, 0);
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ============= Data processing functions ================

float DRV::real_RPM(float angle_deg) {
    int64_t now_us = esp_timer_get_time();
    int64_t dt_us  = now_us - rpm_last_time;

    // Initialize rpm filter
    if (rpm_last_time == 0 || dt_us < RPM_MIN_DT_US) {
        rpm_last_time  = now_us;
        rpm_last_angle = angle_deg;
        return rpm_filtered;
    }

    // Wrap angular delta between -180 and +180
    float delta = angle_deg - rpm_last_angle;
    if      (delta >  RPM_UNWRAP_THR) delta -= 360.0f;
    else if (delta < -RPM_UNWRAP_THR) delta += 360.0f;

    // delta [deg] / 360 [deg/rev] / dt [min]  →  RPM
    float dt_min  = (float)dt_us * (1.0f / 60'000'000.0f);
    float raw_rpm = (delta / 360.0f) / dt_min;

    // Exponential moving average (low pass filter to smooth noise)
    rpm_filtered = RPM_LPF * raw_rpm + (1.0f - RPM_LPF) * rpm_filtered;

    rpm_last_angle = angle_deg;
    rpm_last_time  = now_us;

    return rpm_filtered;
}

float DRV::reducer_angle(float motor_angle) {
    if (!initialized) {
        last_angle  = motor_angle;
        reducer     = 0.0f;          // start relative to zero, not absolute
        initialized = true;
        return reducer;
    }

    // Shortest-path delta on the motor side: always in (-180, +180]
    float delta = fmodf(motor_angle - last_angle, 360.0f);
    if (delta >  180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;

    // Scale down by gear ratio
    reducer += delta / reducer_ratio;

    // Normalize reducer angle to [0, 360)
    reducer = fmodf(reducer, 360.0f);
    if (reducer < 0.0f) reducer += 360.0f;

    last_angle = motor_angle;
    return reducer;
}
