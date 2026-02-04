//===================================================================================================//
//                                            MCP2551 CAN                                            //
//                                            Transceiver                                            //
//===================================================================================================//

#include "esp32_S3.h"
#include "stdio.h"
#include "driver/gpio.h"  //ESP32 GPIO
#include "esp_rom_sys.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CAN_Speed 1  //Mbps
#define RX_BUF_SIZE 1024  //Size of the buffer in bytes
// BaudRate * Delay of interrupt / 10

typedef struct {
    uint32_t timestamp_us;

} CAN_pkg;


class CAN {
public:
    void init();
    float calibration();
    int sendData(uint8_t ID, CAN_pkg);


private:


}