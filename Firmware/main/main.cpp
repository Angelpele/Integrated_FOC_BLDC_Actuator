#include "esp32_S3.h"
#include "DRV8302.h"

DRV8302 drv;

extern "C" void app_main(void)
{
        drv.init();
        drv.setup();
        while(1){
            drv.loop();
        }
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


