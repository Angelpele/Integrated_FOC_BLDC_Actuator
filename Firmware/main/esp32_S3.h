// GENERAL GPIO //
uint8_t MOTOR_ID = 0;

#define USB_DP 20
#define USB_DN 19

#define BOOT 0

#define CAN_RX ((gpio_num_t)44)
#define CAN_TX ((gpio_num_t)43)
#define BAUDRATE 115200

#define PVDD_SENSE 17

// DRV8302 GPIO //
#define A_SENSE 1
#define B_SENSE 2
#define C_SENSE 3

#define INL_A 15
#define INH_A 16
#define INL_B 6
#define INH_B 7
#define INL_C 4
#define INH_C 5

#define DC_CAL 39
#define MPWM 40
#define MOC 41
#define NFAULT 42
#define NOCTW 38

#define GAIN 37
#define SO1 8
#define SO2 9

#define EN_DRV 14

// SENSORS //

#define HALL_A 10
#define HALL_B 11
#define HALL_C 12

#define BMI_SDA 47  // 3 Wire SPI
#define BMI_SCK 35
#define BMI_CS 36

#define MAG_DO ((gpio_num_t)48)   // 3 Wire SPI - One way (SSI)
#define MAG_CSN ((gpio_num_t)18)
#define MAG_CLK ((gpio_num_t)21)

#define NTC 13





