#include "esp32_S3.h"
#include "DRV.h"
#include "MT6701.h"
#include "bmi.h"
#include "MCP2551.h"

// TIMER //
volatile uint32_t isr_counter = 0;

TaskHandle_t focTaskHandle = NULL;
TaskHandle_t fbkTaskHandle = NULL;
TaskHandle_t bmiTaskHandle = NULL;
TaskHandle_t drvTaskHandle = NULL;
TaskHandle_t telemetryTaskHandle = NULL;

// MAIN VARIABLE DECLARATION //
DRV drv;
MT6701 mag;
BMI270 bmi270;
DRV_data drv_pkg;
angle_data fbk_pkg;
BMI_data bmi_pkg;
TelemetryPacket pkg;
CAN CAN_bus;

SemaphoreHandle_t drv_mutex = NULL;
SemaphoreHandle_t fbk_mutex = NULL;
SemaphoreHandle_t bmi_mutex = NULL;

// MAIN FUNCTION DECLARATION //
int initialize();
int calibration();
void start_control_timer();
void create_tasks_once();
void controlISR(void* arg);
void focLoop(void* arg);
void fbkLoop(void* arg);
void drvLoop(void* arg);
void telemetryLoop(void* arg);
void bmiLoop(void* arg);

SystemState system_state = SYSTEM_INIT;
DRV_MotionState motion = MOTION_IDLE;
ControlSys control_sys = MAG;
ControlMode control_mode = MODE_CLOSED_LOOP;

bool debug_mode_enabled = true;
bool included_bmi = false;


extern "C" void app_main(void)
{
    drv_mutex = xSemaphoreCreateMutex();
    fbk_mutex = xSemaphoreCreateMutex();
    bmi_mutex = xSemaphoreCreateMutex();
    configASSERT(drv_mutex);
    configASSERT(fbk_mutex);
    configASSERT(bmi_mutex);

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
                    mag.set_override(control_sys == MAG ? false : true);
                    bmi270.set_override(!included_bmi);
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
                    vTaskDelay(pdMS_TO_TICKS(200));
                    create_tasks_once();
                    start_control_timer();
                    system_state = SYSTEM_RUNNING;

                } else {
                    printf("\nCalibration failed, retrying....\n");
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
    static bool usb_installed = false;
    if (!usb_installed) {
        usb_serial_jtag_driver_config_t usb_cfg = { .tx_buffer_size = 256, .rx_buffer_size = 256 };
        if (usb_serial_jtag_driver_install(&usb_cfg) == ESP_OK) usb_installed = true;
    }
    int status = 0;

    if (drv.init() != DRV_OK) {
        status |= 2;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    mag.init();
    if (included_bmi) {
        if (bmi270.init() != BMI_OK) {
            status |= 4;
        }
    }

    if (CAN_bus.init() != 0) {
        status |= 8;
    }
    return status; 
}

int calibration() {
    drv.setup_mcpwm();
    CAN_bus.test();
    return drv.calibration() + mag.calibration() /*+ bmi270.calibration()*/;
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
        xTaskCreatePinnedToCore(fbkLoop, "fbkLoop", 4096, NULL, 11, &fbkTaskHandle, 0);
        if (included_bmi){
            xTaskCreatePinnedToCore(bmiLoop, "bmiLoop", 4096, NULL, 10, &bmiTaskHandle, 0);
        }
        xTaskCreatePinnedToCore(drvLoop, "drvLoop", 4096, NULL, 10, &drvTaskHandle, 0);
        xTaskCreatePinnedToCore(telemetryLoop, "telemetryLoop", 8192, NULL, 7, &telemetryTaskHandle, 0);
        tasks_created = true;
    }
}

void focLoop(void* arg){
    esp_task_wdt_add(NULL);  // Subscribe to watchdog
    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        float angle;
        xSemaphoreTake(fbk_mutex, portMAX_DELAY);
        angle = fbk_pkg.angle;      // Obtain angle from magnetometer / hall feedback
        xSemaphoreGive(fbk_mutex);

        switch (motion) {
            case MOTION_ALIGNING:
                drv.align(angle).done;
                break;

            case MOTION_RAMPING:
            {
                static bool ramp_reset_done = false;
                if (!ramp_reset_done) {
                    drv.reset_spin_timer();
                    ramp_reset_done = true;
                }
                if (drv.ramp(angle, 2.0f, 20.0f)) {
                    ramp_reset_done = false;   // reset for next ramp
                    motion = MOTION_SPINNING;
                }
                break;
            }

            case MOTION_SPINNING:
            {
                bool spin_elapsed = false;
                switch (control_mode) {
                    case MODE_OPENLOOP_BEMF:
                        spin_elapsed = drv.run_openloop_bemf(20.0f, 50.0f); //Target freq and duty
                        break;
                    case MODE_CLOSED_LOOP:
                        spin_elapsed = drv.closed_spin(angle, 20.0f); //Spinning freq
                        break;
                    case MODE_SVPWM:
                        spin_elapsed = drv.svpwm(angle, 20.0f); //Spinning freq
                        break;
                    case MODE_TRAPEZOIDAL:
                        // Future implementation of trapezoidal control
                    break;
                }
                if (spin_elapsed) {
                    motion = MOTION_IDLE;
                }
                break;
            }
            case MOTION_RAMPING_DOWN:
                if (drv.ramp(angle, 10.0f, 0.5f))   //Target freq and final freq
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

void fbkLoop(void* arg) {
    esp_task_wdt_add(NULL);
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (control_sys == MAG) {
            mag_data temp = mag.getdata();
            xSemaphoreTake(fbk_mutex, portMAX_DELAY);
                fbk_pkg.angle = temp.angle;
                fbk_pkg.status = temp.status;
            xSemaphoreGive(fbk_mutex);
        }
        else if (control_sys == FOC) {
            float angle = drv.get_currents_angle();
            xSemaphoreTake(fbk_mutex, portMAX_DELAY);
                fbk_pkg.angle = angle;
            xSemaphoreGive(fbk_mutex);
        }
        else if (control_sys == HALL) {
            // future implementation of Hall sensor
        }

        esp_task_wdt_reset();
        taskYIELD();
    }
}

void bmiLoop(void* arg) {
    esp_task_wdt_add(NULL);
    if (included_bmi) {
        while (true) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            BMI_data temp = bmi270.getdata();
            xSemaphoreTake(bmi_mutex, portMAX_DELAY);
                bmi_pkg = temp;
            xSemaphoreGive(bmi_mutex);
            esp_task_wdt_reset();
        }
    }
    else {
        vTaskDelete(NULL);          //Delete task if BMI not included
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
    static char buf[300];
    if (!debug_mode_enabled) {
        vTaskDelete(NULL);          // Delete the task if debug mode is disabled
    }
    
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        DRV_data td;
        angle_data tm;
        BMI_data tb;

        if (xSemaphoreTake(drv_mutex, portMAX_DELAY) == pdTRUE) {
            td = drv_pkg;
            xSemaphoreGive(drv_mutex);
        } if (xSemaphoreTake(fbk_mutex, portMAX_DELAY) == pdTRUE) {
            tm = fbk_pkg;
            xSemaphoreGive(fbk_mutex);
        } if (xSemaphoreTake(bmi_mutex, portMAX_DELAY) == pdTRUE) {
            tb = bmi_pkg;
            xSemaphoreGive(bmi_mutex);
        }
        float motor_rpm = drv.real_RPM(tm.angle);

        int len = snprintf(
            buf, sizeof(buf),
            "%d,%.3f,%.3f,%.3f,%.3f,%2d,%3d,%3d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f\n",
            MOTOR_ID,
            td.PVDD,
            td.current1,
            td.current2,
            -(td.current1 + td.current2),
            td.A,
            td.B,
            td.C,
            td.NFAULT_raw,
            td.NOCTW_raw,
            td.temperature,
            tm.angle,
            drv.reducer_angle(tm.angle),
            motor_rpm,
            drv.reducer_RPM(motor_rpm)
            /*tb.accx,
            tb.accy,
            tb.accz,
            tb.gyrx,
            tb.gyry,
            tb.gyrz,
            tb.ffall*/
        );

        if (len > 0) {
            printf("%s", buf);
        }

        uint8_t rxbuf[64] = {0};
        int rxlen = usb_serial_jtag_read_bytes(rxbuf, sizeof(rxbuf) - 1, 0);

        if (rxlen > 0) {
            rxbuf[rxlen] = '\0';
            for (int i = rxlen - 1; i >= 0; i--) {
                if (rxbuf[i] == '\r' || rxbuf[i] == '\n' || rxbuf[i] == ' ')
                    rxbuf[i] = '\0';
                else break;
            }

            printf("RAW CMD: [%s] len=%d\n", rxbuf, rxlen);  // add this
            fflush(stdout);

            char* cmd = strchr((char*)rxbuf, ':');
            if (cmd != NULL) {
                cmd++;
                if      (strncmp(cmd, "FBK:MAG_ENCODER", 15) == 0) control_sys = MAG;
                else if (strncmp(cmd, "FBK:HALL_SENSOR", 15) == 0) control_sys = HALL;
                else if (strncmp(cmd, "FBK:FOC_ANGLE", 13) == 0) control_sys = FOC;
                else if (strncmp(cmd, "CTRL:OPENLOOP_BEMF", 19) == 0) control_mode = MODE_OPENLOOP_BEMF;
                else if (strncmp(cmd, "CTRL:CLOSED_LOOP", 16) == 0) control_mode = MODE_CLOSED_LOOP;
                else if (strncmp(cmd, "CTRL:SVPWM", 10) == 0) control_mode = MODE_SVPWM;
                else if (strncmp(cmd, "CTRL:TRAPEZOIDAL", 17) == 0) control_mode = MODE_TRAPEZOIDAL;
                else if (strcmp(cmd, "IDLE") == 0) motion = MOTION_IDLE;
                else if (strcmp(cmd, "ALIGN") == 0) motion = MOTION_ALIGNING;
                else if (strncmp(cmd, "RUN", 3) == 0){
                    int duration = atoi(strchr((char*)cmd, ':') + 1);
                    drv.set_spin_duration(duration);
                    drv.reset_spin_timer();
                    motion = MOTION_RAMPING;
                    printf("Spin duration set to %d seconds\n", duration);
                    fflush(stdout);
                }
                else if (strcmp(cmd, "BRAKE") == 0) motion = MOTION_RAMPING_DOWN;
                else if (strcmp(cmd, "STOP") == 0) gpio_set_level(EN_DRV, 0);
                else if (strncmp(cmd, "ALPHA:", 6) == 0) {
                    float alpha = atof(cmd + 6);
                    drv.set_alpha(alpha);
                    printf("Alpha set to %.3f\n", alpha);
                    fflush(stdout);
                }
                else if (strncmp(cmd, "BETA:", 5) == 0) {
                    float beta = atof(cmd + 5);
                    drv.set_beta(beta);
                    printf("Beta set to %.3f\n", beta);
                    fflush(stdout);
                }
                else if (strncmp(cmd, "TETA:", 5) == 0) {
                    float teta = atof(cmd + 5);
                    drv.set_teta(teta);
                    printf("Teta set to %.3f\n", teta);
                    fflush(stdout);
                }
                else if (strncmp(cmd, "DUR:", 5) == 0) {
                    float duration = atof(cmd + 5);
                    drv.set_ramp_duration(duration);
                    printf("Ramp duration set to %.3f\n", duration);
                    fflush(stdout);
                }
                else if (strcmp(cmd, "FAULT_OVERRIDE_ON") == 0) drv.set_override(true);
                else if (strcmp(cmd, "FAULT_OVERRIDE_OFF") == 0) drv.set_override(false);
            }
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

    if (tick % FOC_DIVIDER == 0)
        vTaskNotifyGiveFromISR(focTaskHandle, &woken);
    if (tick % FBK_DIVIDER == 0)
        vTaskNotifyGiveFromISR(fbkTaskHandle, &woken);
    if (tick % DRV_DIVIDER == 0)
        vTaskNotifyGiveFromISR(drvTaskHandle, &woken);
    if (tick % TELEMETRY_DIVIDER == 0)
        vTaskNotifyGiveFromISR(telemetryTaskHandle, &woken);

    portYIELD_FROM_ISR(woken);
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


