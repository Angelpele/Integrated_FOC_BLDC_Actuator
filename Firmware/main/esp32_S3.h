#ifndef ESP32_S3_H
#define ESP32_S3_H

// GENERAL GPIO //
#define MOTOR_ID 0;

#define SERIAL_RATE 100   

#define USB_DP 20
#define USB_DN 19

#define BOOT 0

#define CAN_RX ((gpio_num_t)44)
#define CAN_TX ((gpio_num_t)43)
#define BAUDRATE 115200

// DRV8302 GPIO //
#define A_SENSE ADC1_CHANNEL_0  // GPIO 1
#define B_SENSE ADC1_CHANNEL_1  // GPIO 2
#define C_SENSE ADC1_CHANNEL_2  // GPIO 3

#define INL_A ((gpio_num_t)15)
#define INH_A ((gpio_num_t)16)
#define INL_B ((gpio_num_t)6)
#define INH_B ((gpio_num_t)7)
#define INL_C ((gpio_num_t)4)
#define INH_C ((gpio_num_t)5)

#define DC_CAL ((gpio_num_t)39)
#define MPWM ((gpio_num_t)40)
#define MOC ((gpio_num_t)41)
#define NFAULT ((gpio_num_t)42)
#define NOCTW ((gpio_num_t)38)

#define GAIN ((gpio_num_t)37)
#define SO1 ADC1_CHANNEL_7  // GPIO 8
#define SO2 ADC1_CHANNEL_8  // GPIO 9

#define EN_DRV ((gpio_num_t)14)

// SENSORS //

#define HALL_A ((gpio_num_t)10)
#define HALL_B ((gpio_num_t)11)
#define HALL_C ((gpio_num_t)12)

#define BMI_SDA ((gpio_num_t)47) // 3 Wire SPI
#define BMI_SCK ((gpio_num_t)35)
#define BMI_CS ((gpio_num_t)36)

#define MAG_DO ((gpio_num_t)48)   // 3 Wire SPI - One way (SSI)
#define MAG_CSN ((gpio_num_t)18)
#define MAG_CLK ((gpio_num_t)21)

#define NTC ADC2_CHANNEL_2  // GPIO 13
#define PVDD_SENSE ADC2_CHANNEL_6  // GPIO 17

enum SystemState {
    SYSTEM_INIT,
    SYSTEM_CALIBRATION,
    SYSTEM_RUNNING,
};

volatile uint32_t isr_counter = 0;

TaskHandle_t focTaskHandle = NULL;
TaskHandle_t magTaskHandle = NULL;
TaskHandle_t drvTaskHandle = NULL;
TaskHandle_t telemetryTaskHandle = NULL;

#define FOC_DIVIDER        1
#define MAG_DIVIDER    4
#define DRV_DIVIDER        10
#define TELEMETRY_DIVIDER  200

#endif





