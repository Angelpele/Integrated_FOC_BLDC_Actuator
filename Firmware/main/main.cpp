#include "esp32_S3.h"
#include "DRV.h"
#include "MT6701.h"

// TIMER //
volatile uint32_t isr_counter = 0;

TaskHandle_t focTaskHandle = NULL;
TaskHandle_t magTaskHandle = NULL;
TaskHandle_t drvTaskHandle = NULL;
TaskHandle_t telemetryTaskHandle = NULL;

// MAIN VARIABLE DECLARATION //
DRV drv;
MT6701 mag;

DRV_data drv_pkg;
mag_data mag_pkg;

// MAIN FUNCTION DECLARATION //
int initialize();
int calibration();
void start_control_timer();
void create_tasks_once();
void controlISR(void* arg);
void focLoop(void* arg);
void magLoop(void* arg);
void drvLoop(void* arg);
void telemetryLoop(void* arg);

SystemState system_state = SYSTEM_INIT;
bool debug_mode_enabled = false;


extern "C" void app_main(void)
{
    if (!debug_mode_enabled) {
        while (true)
        {
            switch (system_state)
            {
                // ---------------- INIT ----------------
                case SYSTEM_INIT:
                {                 
                    if (initialize() == 0) {
                        system_state = SYSTEM_CALIBRATION;
                        drv.set_override(false);
                        mag.set_override(true);
                    } else {
                        printf("Initialization failed, retrying...\n");
                        vTaskDelay(pdMS_TO_TICKS(200));
                    }
                    break;
                }
                
                // ---------------- CALIBRATION ----------------
                case SYSTEM_CALIBRATION:
                {
                    if (calibration() == 0) {
                        printf("\nRunning");
                        for (int i = 0; i < 10; i++) {
                            printf(".");
                            vTaskDelay(pdMS_TO_TICKS(200));
                        }
                        printf("\n");
                        start_control_timer();
                        system_state = SYSTEM_RUNNING;
                    } else {
                        printf("\n\nCalibration failed, retrying...");
                        for (int i = 0; i < 10; i++) {
                            printf(".");
                            vTaskDelay(pdMS_TO_TICKS(500));
                        }
                        printf("\n");
                        vTaskDelay(pdMS_TO_TICKS(200));
                    }
                    break;
                }
                
                // ---------------- MAIN LOOP ----------------
                case SYSTEM_RUNNING:
                {
                    create_tasks_once();
                    //drv.test_sinu();
                    if (drv.fault_detection() == DRV_LATCHED) {
                        printf("Critical fault detected. Driver disabled. Waiting for reset....\n");
                        system_state = SYSTEM_INIT;
                        gpio_set_level(EN_DRV, 0);
                          
                    }
                    if (mag.fault_detection() != MT6701_OK) {
                        printf("Magnetometer fault detected. Check sensor and magnetic field....\n");
                    }
                    //Software can feed a watchdog timer by writing any value to TIMG_WDTFEED_REG for MDWTs and RTC_CNTL_RTC_WDT_FEED for RWDT
                    vTaskDelay(pdMS_TO_TICKS(10)); //Feed watchdog
                    esp_task_wdt_reset(); // alternative to feed watchdog
                    break;
                }
            }
        }
    } else {
        drv.init();          // 3. Init DRV (now that PWM exists)
        drv.setup_mcpwm();   // 1. Setup PWM hardware first
        mag.init();          // 2. Init MAG
        
        vTaskDelay(pdMS_TO_TICKS(500)); // Small delay to let hardware settle
        
        drv.set_override(true);
        drv.calibration();
        // ---------------------
        
        while(true){
            //mag_pkg = mag.getdata();
            //mag.printdata(mag_pkg);
            //printf("\n");
            drv.update_spwm();
            
            // Note: If update_spwm() generates the motor sine wave, 
            // a 10ms delay here will still ruin the motor commutation.
            // Consider using esp_rom_delay_us(1) or esp_task_wdt_reset() instead.
            vTaskDelay(pdMS_TO_TICKS(10)); 
        }
    }
}

int initialize() {
    drv.setup_mcpwm();
    mag.init();
    
    return drv.init();
}

int calibration() {
    return drv.calibration() + mag.calibration();
}

void start_control_timer()
{
    timer_config_t config = {};
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.auto_reload = TIMER_AUTORELOAD_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.clk_src = TIMER_SRC_CLK_APB;

    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);

    // 1 MHz timer → 1 tick = 1 µs
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_US);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);

    timer_isr_register(
        TIMER_GROUP_0,
        TIMER_0,
        controlISR,
        NULL,
        ESP_INTR_FLAG_IRAM,
        NULL
    );
    timer_start(TIMER_GROUP_0, TIMER_0);
}

void create_tasks_once() {
    static bool tasks_created = false;
    if (!tasks_created) {
        xTaskCreatePinnedToCore(focLoop, "focLoop", 4096, NULL, 10, &focTaskHandle, 1);
        xTaskCreatePinnedToCore(magLoop, "magLoop", 4096, NULL, 9, &magTaskHandle, 1);
        xTaskCreatePinnedToCore(drvLoop, "drvLoop", 4096, NULL, 8, &drvTaskHandle, 1);
        xTaskCreatePinnedToCore(telemetryLoop, "telemetryLoop", 4096, NULL, 2, &telemetryTaskHandle, 0);
        tasks_created = true;
    }
}

void focLoop(void* arg){
    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        //drv.update_spwm();      // PWM update
    }
}

void magLoop(void* arg){
    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        mag_pkg = mag.getdata();
    }
}

void drvLoop(void* arg){
    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        drv_pkg = drv.getdata();
    }
}

void telemetryLoop(void* arg){
    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        drv.printdata(drv_pkg);
        mag.printdata(mag_pkg);
        printf("\n");
    }
}

void IRAM_ATTR controlISR(void* arg)
{
    // Clear the timer interrupt using the proper API
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);

    // After an alarm, the hardware disables it. You must re-enable it for the next tick.
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    isr_counter += FOC_DIVIDER;
    vTaskNotifyGiveFromISR(focTaskHandle, &xHigherPriorityTaskWoken);

    if(isr_counter % MAG_DIVIDER == 0)
        vTaskNotifyGiveFromISR(magTaskHandle, &xHigherPriorityTaskWoken);

    if(isr_counter % DRV_DIVIDER == 0)
        vTaskNotifyGiveFromISR(drvTaskHandle, &xHigherPriorityTaskWoken);

    if(isr_counter % TELEMETRY_DIVIDER == 0)
        vTaskNotifyGiveFromISR(telemetryTaskHandle, &xHigherPriorityTaskWoken);

    if(xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR();
}


/*main
|
|----> init :  initalize all sensors + components + timers
|----> calib : calibrate rest of components
|
|----> Open loop mov
        --> fast loop BLDC readings: CORE 1
|----> Closed loop mov
        --> fast loop BLDC + MAG readings: CORE 1
|
|----> Sensors data store: CORE 0 (lower freq)

|-> Outer Control Loop:
    ---> Call mov funcs with input pos, vel, acc


|----> BL Comms func
|----> CAN Comms func: packages to CAN protocol
|----> Server web info print


ERROR Flags can stop the program exec


BMI.c funcs:

init (define pins + protocols)
calibration
pull data

MAG.c funcs:

init (define pins + protocols)
calibration
pull data

DRV.c funcs:

init (define pins + protocols)
calibration
pull data

ERROR flags

ESP32 funcs

NTC, voltages... data




*/


