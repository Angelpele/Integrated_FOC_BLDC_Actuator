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
    gpio_set_level(GAIN, 0);          // choose gain explicitly
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
    //SO1_raw = adc1_get_raw(SO1);
    //SO2_raw = adc1_get_raw(SO2);
    //SO1_raw = 0;
    //SO2_raw = 0;
    //package.current1 = (float)(SO1_offset - SO1_raw) * voltage_step / (current_gain * R_shunt);
    //package.current2 = (float)(SO2_offset - SO2_raw) * voltage_step / (current_gain * R_shunt);
    package.current1 = i_q_filt;
    package.current2 = i_d_filt;
    
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
extern "C" void mcpwm_isr(void* arg);  // Declare the ISR for mcpwm in main.cpp

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

    esp_intr_alloc(ETS_PWM0_INTR_SOURCE, ESP_INTR_FLAG_IRAM, mcpwm_isr, NULL, NULL);
    MCPWM0.int_ena.timer0_tep_int_ena = 1; // Enable interrupt on Timer 0 TEP (Timer Event Peak)
}

bool DRV::svpwm(float v_a, float v_b, float v_c) {
    // Space Vector PWM implementation
    // v_a/v_b/v_c are phase voltages relative to the DC midpoint [Volts].

    int raw_pvdd = 0;
    adc2_get_raw(PVDD_SENSE, ADC_WIDTH_BIT_12, &raw_pvdd);
    float v_bus = getPVDD(raw_pvdd);
    if (v_bus < 4.0f) v_bus = 4.0f;     // safety floor – avoids /0 at startup
    float v_half = v_bus * 0.5f;

    //Third-harmonic (min-max) zero-sequence injection
    float v_min = fminf(fminf(v_a, v_b), v_c);
    float v_max = fmaxf(fmaxf(v_a, v_b), v_c);
    float v_zero = -0.5f * (v_max + v_min);   // centred zero-sequence

    v_a += v_zero;
    v_b += v_zero;
    v_c += v_zero;

    // Clamp to ±v_half to avoid saturation (should not happen after voltage
    // clamping in foc_control, but kept as a hard guard)
    v_a = fmaxf(-v_half, fminf(v_half, v_a));
    v_b = fmaxf(-v_half, fminf(v_half, v_b));
    v_c = fmaxf(-v_half, fminf(v_half, v_c));


    float duty_a = (v_a / v_half + 1.0f) * 50.0f;
    float duty_b = (v_b / v_half + 1.0f) * 50.0f;
    float duty_c = (v_c / v_half + 1.0f) * 50.0f;

    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_A, duty_a);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_B, 100.0f - duty_a);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_A, duty_b);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_B, 100.0f - duty_b);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_A, duty_c);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_B, 100.0f - duty_c);
    return false;
}

AlignOffset DRV::align(float angle) {
    AlignOffset result = {false, 0.0f};
    float align_voltage = (align_time < ramp_duration) ? (align_time / ramp_duration) * 6.0f : 6.0f;
    
    svpwm(align_voltage, -align_voltage / 2.0f, -align_voltage / 2.0f);
    
    align_time += dt;

    if (align_time >= ALIGN_DURATION) {
        float theta_mag = angle * (M_PI / 180.0f);
        
        elec_offset = - (theta_mag * (float)MOTOR_POLE_PAIRS);

        while (elec_offset >  M_PI) elec_offset -= 2.0f * M_PI;
        while (elec_offset < -M_PI) elec_offset += 2.0f * M_PI;

        phase_accumulator = 0.0f;
        align_time = 0.0f; 

        printf("Alignment done. Angle: %.2f deg | elec_offset: %.4f rad\n", angle, elec_offset);
        
        result.done = true;
        result.elec_offset = elec_offset;
    }
    just_aligned = true;
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

    float step = ramp_rate * dt;
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

    float t = fabsf(electrical_freq - init_freq) / fabsf(obj_freq - init_freq + 1e-6f);
    float blend    = t * t * (3.0f - 2.0f * t); // Smoothstep for better transition, s curve equation

    // Low pass filter for angle estimation during ramping, same as spinning
    static float theta_elec_lpf = 0.0f;
    if (theta_reset) { theta_elec_lpf = phase_accumulator; theta_reset = false; }
    theta_elec_lpf = theta_elec_lpf + CLOSED_LOOP_LPF * (theta_elec_raw - theta_elec_lpf);

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

// ============= Control functions ================

bool DRV::sinusoidal_spin(float fbk_angle, float freq) {

    if (spin_duration > 0.0f) {
        spin_elapsed += dt;
        if (spin_elapsed >= spin_duration) {
            spin_elapsed = 0.0f;
            theta_reset = true;
            return true;
        }
    }

    electrical_freq = freq;
    float theta_mag = fbk_angle * (M_PI / 180.0f);
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

bool DRV::foc_control(float fbk_angle)
{
    if (foc_stop) {
        reset_foc();
        return true;
    }

    float now = (float)esp_timer_get_time() * 1e-6f;
    if (!foc_initialized) {
        reset_foc();
        foc_initialized = true;
        foc_prev_time   = now;
        return false; 
    }

    float dt = now - foc_prev_time;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.000050f;  // clamp to 50µs

    // Current sensing is done sync with MCPWM

    float i_a = (float)(SO1_raw - SO1_offset) * voltage_step / (current_gain * R_shunt);
    float i_b = (float)(SO2_raw - SO2_offset) * voltage_step / (current_gain * R_shunt);

    // Mechanical angle to electrical angle
    float theta_raw = fbk_angle * (M_PI / 180.0f);
    float theta = fmodf(theta_raw * (float)MOTOR_POLE_PAIRS + elec_offset, 2.0f * M_PI);
    if (theta < 0.0f) theta += 2.0f * M_PI;

    if (just_aligned){
        printf("Check: theta after align = %.3f (should be ~0)\n", theta);
        just_aligned = false;
    }
    static int debug_count = 0;
    if (debug_count < 10) {
        float theta_raw = fbk_angle * (M_PI / 180.0f);
        float theta_check = fmodf(theta_raw * (float)MOTOR_POLE_PAIRS + elec_offset, 2.0f * M_PI);
        printf("FOC debug: fbk_angle=%.2f elec_offset=%.4f theta=%.3f\n", 
               fbk_angle, elec_offset, theta_check);
        debug_count++;
    }


    float cos_t = cosf(theta);
    float sin_t = sinf(theta);

    // Clarke transform (2 sensors, balanced 3 phase assumed)
    float i_alpha =  i_a;
    float i_beta  = (i_a + 2.0f * i_b) * (1.0f / 1.7320508f);  // Divide by sqrt(3)

    // Park transform
    float i_d =  i_alpha * cos_t + i_beta * sin_t;
    float i_q = -i_alpha * sin_t + i_beta * cos_t;

    // Low-pass filter
    i_d_filt += Id_LPF * (i_d - i_d_filt);
    i_q_filt += Iq_LPF * (i_q - i_q_filt);

    // Current errors
    float error_q = target_current - i_q_filt;
    float error_d = 0.0f - i_d_filt;

    // Integrate before updating prev_time
    errorq_integral += error_q * dt;
    errord_integral += error_d * dt;

    int raw_pvdd = 0;
    adc2_get_raw(PVDD_SENSE, ADC_WIDTH_BIT_12, &raw_pvdd);
    float v_bus   = getPVDD(raw_pvdd);
    if (v_bus < 4.0f) v_bus = 4.0f;
    // Maximum phase voltage magnitude = V_bus / sqrt(3)  (SVPWM limit)
    float v_limit = fminf(v_bus / sqrtf(3.0f), overvoltage);

    // Anti-windup clamp on integrals
    float int_limit_q = v_limit / (Iq_Ki + 1e-9f);
    float int_limit_d = v_limit / (Id_Ki + 1e-9f);
    errorq_integral = fmaxf(-int_limit_q, fminf(int_limit_q, errorq_integral));
    errord_integral = fmaxf(-int_limit_d, fminf(int_limit_d, errord_integral));

    // PID
    float v_q = Iq_Kp * error_q + Iq_Ki * errorq_integral + Iq_Kd * (error_q - errorq_prev) / dt;
    float v_d = Id_Kp * error_d + Id_Ki * errord_integral + Id_Kd * (error_d - errord_prev) / dt;

    float v_dq_mag = sqrtf(v_d * v_d + v_q * v_q);
    if (v_dq_mag > v_limit) {
        float scale = v_limit / v_dq_mag;
        v_d *= scale;
        v_q *= scale;
    }

    // Update state for next cycle
    foc_prev_time = now;
    errorq_prev   = error_q;
    errord_prev   = error_d;

    // Inverse Park
    float v_alpha = v_d * cos_t - v_q * sin_t;
    float v_beta  = v_d * sin_t + v_q * cos_t;

    // Inverse Clarke
    float v_a =  v_alpha;
    float v_b = -0.5f * v_alpha + (sqrtf(3.0f) / 2.0f) * v_beta;
    float v_c = -0.5f * v_alpha - (sqrtf(3.0f) / 2.0f) * v_beta;

    return svpwm(v_a, v_b, v_c);
}

float DRV::velocity_loop (float fbk_rpm) {
    float now_vel = (float)esp_timer_get_time() * 1e-6f;
    if (!vel_initialized) {
        reset_vel_loop();
        vel_initialized = true;
        vel_prev_time = now_vel;
        return false; 
    }

    float dt_vel = now_vel - vel_prev_time;
    if (dt_vel <= 0.0f) return target_current;
    
    fbk_rpm_filt = fbk_rpm_filt + Vel_LPF * (fbk_rpm - fbk_rpm_filt);
    
    float d_error = target_vel - fbk_rpm_filt;
    vel_error_integral += d_error * dt_vel;

    // Anti-windup
    float int_limit = VEL_OUT_MAX / (Vel_Ki + 1e-9f); // Prevent integral from commanding more than max current
    vel_error_integral = fmaxf(-int_limit, fminf(int_limit, vel_error_integral));

    float output = Vel_Kp * d_error + Vel_Ki * vel_error_integral + Vel_Kd * (d_error - vel_error_prev) / dt_vel;
    output = fmaxf(-VEL_OUT_MAX, fminf(VEL_OUT_MAX, output)); // Clamp to max current
    vel_error_prev = d_error;
    vel_prev_time = now_vel;
    target_current = output; // Feed velocity controller output to current loop as target
    return output;
}

float DRV::position_loop(float fbk_angle_deg) {
    float now_pos = (float)esp_timer_get_time() * 1e-6f;
    if (!pos_initialized) {
        reset_pos_loop();
        pos_initialized = true;
        pos_prev_time = now_pos;
        return false; 
    }

    float dt_pos = now_pos - pos_prev_time;
    if (dt_pos <= 0.0f || dt_pos > 0.5f){
        pos_prev_time = now_pos;
        return target_vel;
    } 

    
    float raw_error = target_pos - fbk_angle_deg;

    // Cast angle if NO reducer
    
    //if (raw_error > 180.0f) raw_error -= 360.0f;
    //if (raw_error < -180.0f) raw_error += 360.0f;
    
    pos_error_filt += Pos_LPF * (raw_error - pos_error_filt);

    pos_error_integral += pos_error_filt * dt_pos;

    // Anti-windup
    float int_limit = POS_OUT_MAX / (Pos_Ki + 1e-9f); // 
    pos_error_integral = fmaxf(-int_limit, fminf(int_limit, pos_error_integral));

    float output = Pos_Kp * pos_error_filt + Pos_Ki * pos_error_integral + Pos_Kd * (pos_error_filt - pos_error_prev) / dt_pos;

    //VIRTUAL LIMIT SWITCH FOR REDUCER
    if (reducer_limits_enabled) {
        if (fbk_angle_deg >= reducer_max_angle && output > 0.0f) {
            output = 0.0f;              // Kill the forward command
            pos_error_integral = 0.0f;  // Prevent integral windup against the invisible wall
        }
        else if (fbk_angle_deg <= reducer_min_angle && output < 0.0f) {
            output = 0.0f;              // Kill the backward command
            pos_error_integral = 0.0f;  // Prevent integral windup
        }
    }
    
    output = fmaxf(-POS_OUT_MAX, fminf(POS_OUT_MAX, output));   // Clamp to max velocity
    pos_error_prev = pos_error_filt;
    pos_prev_time = now_pos;
    target_vel = output; // Feed position controller output to velocity loop as target
    return output;
}

bool DRV::openloop_bemf(float target_freq, float duty) {
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
        //return svpwm(phase_accumulator * (180.0f / M_PI), electrical_freq);
        return true;
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
    if (rpm_last_time == 0) {
        rpm_last_time  = now_us;
        rpm_last_angle = angle_deg;
        return rpm_filtered;
    }
    if (dt_us < RPM_MIN_DT_US) {
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
    //rpm_filtered = RPM_LPF * raw_rpm + (1.0f - RPM_LPF) * rpm_filtered;
    rpm_filtered = raw_rpm; // No filtering for now, just raw value
    rpm_last_angle = angle_deg;
    rpm_last_time  = now_us;

    return raw_rpm;
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

    // Normalize reducer angle to [0, 360) if only motor with no reducer
    //reducer = fmodf(reducer, 360.0f);
    // if (reducer < 0.0f) reducer += 360.0f;

    last_angle = motor_angle;
    return reducer;
}
