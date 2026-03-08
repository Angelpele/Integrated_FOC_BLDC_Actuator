#include "esp32_S3.h"
#include "DRV.h"
#include "MT6701.h"

DRV drv;
MT6701 mag;

DRV_data drv_pkg;
mag_data mag_pkg;

int initialize();
int calibration();

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
                        printf("\nRunning")
                        for(int i=0; i<10; i++) {
                            printf(".");
                            vTaskDelay(pdMS_TO_TICKS(200));
                        printf("\n");
                        start_control_timer();
                        system_state = SYSTEM_RUNNING;
                    } else {
                        printf("\n\nCalibration failed, retrying...");
                        for(int i=0; i<10; i++) {
                            printf(".");
                            vTaskDelay(pdMS_TO_TICKS(500));
                        }printf("\n");
                        vTaskDelay(pdMS_TO_TICKS(200));
                    }
                    break;
                }

                // ---------------- MAIN LOOP ----------------
                case SYSTEM_RUNNING:
                {
                    xTaskCreatePinnedToCore(focLoop, "focLoop", 4096, NULL, 10, &focTaskHandle, 1);
                    xTaskCreatePinnedToCore(magLoop, "magLoop", 4096, NULL, 9, &magTaskHandle, 1);
                    xTaskCreatePinnedToCore(drvLoop, "drvLoop", 4096, NULL, 8, &drvTaskHandle, 1);
                    xTaskCreatePinnedToCore(telemetryLoop, "telemetryLoop", 4096, NULL, 2, &telemetryTaskHandle, 0);
                    //function executed, memory allocated, pointer, priority, pointer handle, core number)
                    
                    //drv.test_sinu();
                    if (drv.fault_detection() == DRV_LATCHED) {
                        printf("Critical fault detected. Driver disabled. Waiting for reset....\n");
                        gpio_set_level(EN_DRV, 0);
                    }
                    break;
                }
            }
        }
    } else {
        mag.init();
        mag.calibration();
        while(true){
            mag_pkg = mag.getdata();
            //uint32_t raw = mag.getraw();
    
            //vTaskDelay(pdMS_TO_TICKS(SERIAL_RATE));
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

void start_control_timer(){
    hw_timer_t* timer = NULL;

    timer = timerBegin(0, 80, true); 
    // timer num, prescaler, count up 
    // 1 MHz timer (prescaler = 80), clk of the esp 80Mhz?
    timerAttachInterrupt(timer, &controlISR, true);
    //timer, function triggered by interrupt, edge
    timerAlarmWrite(timer, 50, true);  
    // 1 / 50 µs → 20 kHz. Timer, ticks, autoreload. Timer runs at 1MHz, each 1µs
    timerAlarmEnable(timer);
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

void IRAM_ATTR controlISR()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    isr_counter+=FOC_DIVIDER;
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


