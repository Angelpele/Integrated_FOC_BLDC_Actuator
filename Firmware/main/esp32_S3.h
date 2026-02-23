#ifndef ESP32_S3_H
#define ESP32_S3_H

// GENERAL GPIO //
#define MOTOR_ID 0;

#define USB_DP 20
#define USB_DN 19

#define BOOT 0

#define CAN_RX ((gpio_num_t)44)
#define CAN_TX ((gpio_num_t)43)
#define BAUDRATE 115200

#define PVDD_SENSE 17

// DRV8302 GPIO //
#define A_SENSE ((gpio_num_t)1)
#define B_SENSE ((gpio_num_t)2)
#define C_SENSE ((gpio_num_t)3)

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
#define SO1 ((gpio_num_t)8)
#define SO2 ((gpio_num_t)9)

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

#define NTC ((gpio_num_t)13)

#endif





