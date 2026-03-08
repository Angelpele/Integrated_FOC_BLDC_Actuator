//===================================================================================================//
//                                            MCP2551 CAN                                            //
//                                            Transceiver                                            //
//===================================================================================================//

#include "MCP2551.h"


void CAN::init()
{
    gpio_set_direction(CAN_RX, GPIO_MODE_INPUT);
    gpio_set_direction(CAN_TX, GPIO_MODE_OUTPUT);

    const uart_config_t uart_config = {
        .baud_rate = BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,  //Not required for HIGH Baudrate
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, RX_BUF_SIZE * 2, 0, NULL, 0);  
        //BUff size for RX, for TX, required if CAN interrupted by program
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, CAN_TX, CAN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);    
        //Number defined for CAN pins, NO_CHANGE for pins not used (CTS and RTS pins)   
}

int CAN::sendData(uint8_t ID, CAN_pkg data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_0, data, len);
    ESP_LOGI(ID, "Wrote %d bytes", txBytes);
    return txBytes;
}

void CAN::tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        sendData(TX_TASK_TAG, "Hello world");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void CAN::rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_0, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(data);
}