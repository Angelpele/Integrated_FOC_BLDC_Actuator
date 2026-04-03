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
    adc1_config_channel_atten(C_SENSE, ADC_ATTEN_DB_12);
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

    printf("DRV Offsets -> SO1: %d | SO2: %d\n", SO1_offset, SO2_offset);

    // Basic sanity check: Offset should be roughly middle of 12-bit range (2048)
    if (SO1_offset < 1000 || SO1_offset > 3000) {
        printf("WARNING: SO1 Offset unusual. Check hardware!\n");
        return -1; 
    }
    printf("DRV CALIBRATION COMPLETE\n");
    return 0; // Return 0 on success
}

DRV_data DRV::getdata() 
{
    DRV_data package;

    // ---- Digital inputs ----
    package.NFAULT_raw = gpio_get_level(NFAULT);
    package.NOCTW_raw = gpio_get_level(NOCTW);
    
    // ---- ADC1 ----
    package.A_raw = adc1_get_raw(A_SENSE);
    package.B_raw = adc1_get_raw(B_SENSE);
    package.C_raw = adc1_get_raw(C_SENSE);
    //SO1_raw = adc1_get_raw(SO1);
    //SO2_raw = adc1_get_raw(SO2);
    SO1_raw = 0;
    SO2_raw = 0;
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

int DRV::fault_detection()
{
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
            latch_counter--;
    }

    if (latch_counter >= 100) {
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

void DRV::update_spwm(float angle)
{
    // =========================
    // 1. ALIGNMENT (OPEN-LOOP) + OFFSET CALIBRATION
    // =========================
    if (!aligned) {
        const float ALIGN_DUTY = 40.0f;

        // Soft-start ramp
        float duty = (align_time < ramp_duration) ? (align_time / ramp_duration) * ALIGN_DUTY : ALIGN_DUTY;

        // Force rotor to electrical zero: A high, B low, C floating
        mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
        mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_B, 100.0f - duty);
        mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
        mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_B, 100);
        mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_A, 0);
        mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_B, 100);

        align_time += dt;

        if (align_time >= ALIGN_DURATION) {
            float theta_mag = angle * (M_PI / 180.0f);
            elec_offset = -theta_mag * (float)MOTOR_POLE_PAIRS;
            
            while (elec_offset >  M_PI) elec_offset -= 2.0f * M_PI; // Normalise to (-π, π]
            while (elec_offset < -M_PI) elec_offset += 2.0f * M_PI;
            phase_accumulator = 0.0f;   // we are at electrical zero by definition

            offset_calibrated = true;
            aligned = true;
            printf("Alignment done. Angle: %.2f deg | elec_offset: %.4f rad\n", angle, elec_offset);
        }
        esp_task_wdt_reset();
        return;
    }

    // =========================
    // 2. SENSOR ELECTRICAL ANGLE
    // =========================
    float theta_mag = angle * (M_PI / 180.0f);
    float theta_elec_raw = theta_mag * (float)MOTOR_POLE_PAIRS + elec_offset;

    // Normalise to [0, 2π)
    theta_elec_raw = fmodf(theta_elec_raw, 2.0f * M_PI);
    if (theta_elec_raw < 0.0f) theta_elec_raw += 2.0f * M_PI;

    // =========================
    // 3. FREQUENCY RAMP
    // =========================
    if (!ramp_done) {
        electrical_freq += ramp_rate * dt;
        if (electrical_freq >= target_electrical_freq) {
            electrical_freq = target_electrical_freq;
            ramp_done = true;
        }
        phase_accumulator += 2.0f * M_PI * electrical_freq * dt;
        if (phase_accumulator >= 2.0f * M_PI) phase_accumulator -= 2.0f * M_PI;
        if (phase_accumulator < 0.0f)         phase_accumulator += 2.0f * M_PI;

        float blend = electrical_freq / target_electrical_freq;
        float theta_elec  = (1.0f - blend) * phase_accumulator + blend * theta_elec_raw;

        float duty_a = (sinf(theta_elec) * modulation_index + 1.0f) * 50.0f;
        float duty_b = (sinf(theta_elec - 2.0f * M_PI / 3.0f) * modulation_index + 1.0f) * 50.0f;
        float duty_c = (sinf(theta_elec - 4.0f * M_PI / 3.0f) * modulation_index + 1.0f) * 50.0f;

        mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_A, duty_a);
        mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_B, 100.0f - duty_a);
        mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_A, duty_b);
        mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_B, 100.0f - duty_b);
        mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_A, duty_c);
        mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_B, 100.0f - duty_c);
        return;
    }

    // =========================
    // 4. CLOSED-LOOP (sensor only)
    // =========================
    float duty_a = (sinf(theta_elec_raw) * modulation_index + 1.0f) * 50.0f;
    float duty_b = (sinf(theta_elec_raw - 2.0f * M_PI / 3.0f) * modulation_index + 1.0f) * 50.0f;
    float duty_c = (sinf(theta_elec_raw - 4.0f * M_PI / 3.0f) * modulation_index + 1.0f) * 50.0f;

    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_A, duty_a);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_B, 100.0f - duty_a);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_A, duty_b);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_B, 100.0f - duty_b);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_A, duty_c);
    mcpwm_set_duty(GPIO_PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_B, 100.0f - duty_c);
}

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