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

SemaphoreHandle_t drv_mutex = NULL;
SemaphoreHandle_t mag_mutex = NULL;

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
DRV_MotionState motion = MOTION_ALIGNING;
bool debug_mode_enabled = true;


extern "C" void app_main(void)
{
    drv_mutex = xSemaphoreCreateMutex();
    mag_mutex = xSemaphoreCreateMutex();
    configASSERT(drv_mutex);
    configASSERT(mag_mutex);

    esp_task_wdt_add(NULL); // Subscribe main to the task watchdog

    while (true)
    {
        switch (system_state)
        {
            // ---------------- INIT ----------------
            case SYSTEM_INIT:
            {       
                
                gpio_set_pull_mode(CAN_RX, GPIO_PULLUP_ONLY); // GPIO44
                gpio_set_pull_mode(CAN_TX, GPIO_PULLUP_ONLY); // GPIO43

                if (initialize() == 0) {
                    system_state = SYSTEM_CALIBRATION;
                    drv.set_override(true);
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
                    printf("\nRunning....\n");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    create_tasks_once();
                    start_control_timer();
                    system_state = SYSTEM_RUNNING;

                } else {
                    printf("\n\nCalibration failed, retrying....\n");
                    vTaskDelay(pdMS_TO_TICKS(2000));
                }
                break;
            }
            
            // ---------------- MAIN LOOP ----------------
            case SYSTEM_RUNNING:
            {;
                esp_task_wdt_reset();
                if (drv.fault_detection() == DRV_LATCHED) {
                    printf("Critical fault detected. Driver disabled. Waiting for reset....\n");
                    fflush(stdout);
                    system_state = SYSTEM_INIT;
                    gpio_set_level(EN_DRV, 0);
                    
                }
                vTaskDelay(pdMS_TO_TICKS(10));
                break;
            }
        }
    }
}

int initialize() {
    mag.init();
    return drv.init();
}

int calibration() {
    drv.setup_mcpwm();
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
        xTaskCreatePinnedToCore(focLoop, "focLoop", 4096, NULL, 12, &focTaskHandle, 1);  //1 and 0 are the core IDs
        xTaskCreatePinnedToCore(magLoop, "magLoop", 4096, NULL, 11, &magTaskHandle, 0);
        xTaskCreatePinnedToCore(drvLoop, "drvLoop", 4096, NULL, 10, &drvTaskHandle, 0);
        xTaskCreatePinnedToCore(telemetryLoop, "telemetryLoop", 4096, NULL, 7, &telemetryTaskHandle, 0);
        tasks_created = true;
    }
}

void focLoop(void* arg){
    esp_task_wdt_add(NULL);  // Subscribe to watchdog
    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        float angle;
        xSemaphoreTake(mag_mutex, portMAX_DELAY);
        angle = mag_pkg.angle;
        xSemaphoreGive(mag_mutex);

        switch (motion) {
            case MOTION_ALIGNING:
                if (drv.align(angle).done)
                    motion = MOTION_RAMPING;
                break;

            case MOTION_RAMPING:
                if (drv.ramp(angle, 2.0f, 10.0f, 1.5f))   //Initial freq and target freq
                    motion = MOTION_SPINNING;
                break;

            case MOTION_SPINNING:
                if (drv.spin(angle, 10.0f, 3.0f)) // Spinning freq
                    
                break;

            case MOTION_RAMPING_DOWN:
                if (drv.ramp(angle, 10.0f, 0.5f, 1.5f))   //Target freq and final freq
                    motion = MOTION_IDLE;
                break;

            case MOTION_IDLE:
                drv.brake();
                break;

            default: break;
        }
        esp_task_wdt_reset();
    }
}

void magLoop(void* arg) {
    esp_task_wdt_add(NULL);
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        mag_data temp = mag.getdata();
        xSemaphoreTake(mag_mutex, portMAX_DELAY);
            mag_pkg = temp;
        xSemaphoreGive(mag_mutex);
        esp_task_wdt_reset();
    }
}

void drvLoop(void* arg) {
    esp_task_wdt_add(NULL);
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        DRV_data temp = drv.getdata();
        xSemaphoreTake(drv_mutex, portMAX_DELAY);
            drv_pkg = temp;
        xSemaphoreGive(drv_mutex);
        esp_task_wdt_reset();
    }
}

void telemetryLoop(void* arg) {
    if (debug_mode_enabled) {
        esp_task_wdt_add(NULL);
        static char buf[192];
    
        while (true) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            DRV_data td;
            mag_data tm;
    
            if (xSemaphoreTake(drv_mutex, portMAX_DELAY) == pdTRUE) {
                td = drv_pkg;
                xSemaphoreGive(drv_mutex);
            } if (xSemaphoreTake(mag_mutex, portMAX_DELAY) == pdTRUE) {
                tm = mag_pkg;
                xSemaphoreGive(mag_mutex);
            }
            snprintf(buf, sizeof(buf),
                "PVDD:%.2fV | I1:%.2f I2:%.2f I3:%.2f | A:%d B:%d C:%d | NFault:%d OC:%d | NTC:%.1f | Ang:%.2fº State:%d\n",
                td.PVDD,
                td.current1, td.current2, -(td.current1 + td.current2),
                td.A_raw, td.B_raw, td.C_raw,
                td.NFAULT_raw, td.NOCTW_raw, td.temperature,
                tm.angle, tm.status);
    
            printf("%s", buf);
            esp_task_wdt_reset();
        }
    }
}

void IRAM_ATTR controlISR(void* arg)
{
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);

    static uint32_t tick = 0;
    tick++;

    BaseType_t woken = pdFALSE;

    vTaskNotifyGiveFromISR(focTaskHandle, &woken);

    // Resto: subdivisiones exactas
    if (tick % MAG_DIVIDER == 0)
        vTaskNotifyGiveFromISR(magTaskHandle, &woken);

    if (tick % DRV_DIVIDER == 0)
        vTaskNotifyGiveFromISR(drvTaskHandle, &woken);

    if (tick % TELEMETRY_DIVIDER == 0)
        vTaskNotifyGiveFromISR(telemetryTaskHandle, &woken);

    if (woken) portYIELD_FROM_ISR();
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


