//===================================================================================================//
//                                          BMI270 IMU + Gyro                                        //
//                                            16 bit 3 axis                                          //
//===================================================================================================//
#include "esp32_S3.h"
#include "bmi.h"

static spi_device_handle_t s_spi_handle = nullptr;
 
static BMI2_INTF_RETURN_TYPE bmi2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    spi_device_handle_t spi = *static_cast<spi_device_handle_t *>(intf_ptr);

    spi_transaction_t t = {};
    t.addr      = reg_addr | 0x80;
    t.length    = 0;          // no tx data (address handled by addr field)
    t.rxlength  = len * 8;
    t.rx_buffer = reg_data;

    esp_err_t ret = spi_device_polling_transmit(spi, &t);
    return (ret == ESP_OK) ? BMI2_INTF_RET_SUCCESS : -1;
}
 
static BMI2_INTF_RETURN_TYPE bmi2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    spi_device_handle_t spi = *static_cast<spi_device_handle_t *>(intf_ptr);

    spi_transaction_t t = {};
    t.addr = reg_addr & 0x7F;      // Write bit cleared
    t.length = len * 8;            // Total bits to transmit
    t.tx_buffer = reg_data;        // Data to write
    esp_err_t ret = spi_device_polling_transmit(spi, &t);

    return (ret == ESP_OK) ? BMI2_INTF_RET_SUCCESS : -1;
}

static void bmi2_delay_us(uint32_t period, void * /*intf_ptr*/)
{
    if (period <= 1000) {
        esp_rom_delay_us(period);
    } else {
        vTaskDelay(pdMS_TO_TICKS(period / 1000));
    }
}


BMI_Status BMI270::init() {
    gpio_set_direction(BMI_CS,  GPIO_MODE_OUTPUT);
    gpio_set_level(BMI_CS, 1);          // CS active (active low), toggled by SPI
    
    for(int i=0; i<16; i++) {
        gpio_set_level(BMI_SCK, 1);
        esp_rom_delay_us(10);
        gpio_set_level(BMI_SCK, 0);
        esp_rom_delay_us(10);
    }
    
    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num   = BMI_SDA;
    bus_cfg.miso_io_num   = -1;    // shared data line (3-wire)
    bus_cfg.sclk_io_num   = BMI_SCK;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 64;
    
    
    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.command_bits    = 0;
    dev_cfg.address_bits    = 8;        // address sent manually in tx buffer
    dev_cfg.dummy_bits      = 0;
    dev_cfg.mode            = 0;        
    dev_cfg.clock_speed_hz  = 1 * 1000 * 1000;   // 2 MHz (safe for config phase)
    dev_cfg.spics_io_num    = BMI_CS;
    dev_cfg.queue_size      = 1;
    dev_cfg.flags           = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY;
    
    static bool spi_initialized = false;
    
    // Initialize the SPI bus
    if (!spi_initialized) {
        spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
        spi_initialized = true;
    }

    esp_err_t ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &s_spi_handle);
    if (ret != ESP_OK) {
        printf("BMI270 SPI add device failed: %d\n", ret);
        return BMI_ERR;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    dev = {};                               // zero-initialise the whole struct
    dev.intf      = BMI2_SPI_INTF;          // tells the driver dummy_byte = 1
    dev.read      = bmi2_spi_read;
    dev.write     = bmi2_spi_write;
    dev.delay_us  = bmi2_delay_us;
    dev.intf_ptr  = &s_spi_handle;          // passed back in every callback
    dev.read_write_len = 32;

    uint8_t spi3_config = 0x01; 
    bmi2_set_regs(0x41, &spi3_config, 1, &dev); 

    // Verify chip id before init
    uint8_t chip_id = 0;
    bmi2_get_regs(BMI2_CHIP_ID_ADDR, &chip_id, 1, &dev);
    printf("BMI270 CHIP ID: 0x%02X\n", chip_id);


    int8_t rslt = bmi270_init(&dev);
    if (rslt != BMI2_OK) {
        printf("BMI270 init failed: %d\n", rslt);
        return BMI_ERR;
    }
 
    struct bmi2_sens_config config[2] = {};
    config[0].type = BMI2_ACCEL;
    config[1].type = BMI2_GYRO;
 
    rslt = bmi270_get_sensor_config(config, 2, &dev);
    if (rslt != BMI2_OK) {
        printf("BMI270 get config failed: %d\n", rslt);
        return BMI_ERR;
    }
 
    config[0].cfg.acc.range       = BMI2_ACC_RANGE_2G;  // Lowest value for acc
    config[0].cfg.acc.odr         = BMI2_ACC_ODR_400HZ; // 200, 400, 800, 1600
    config[0].cfg.acc.bwp         = BMI2_ACC_NORMAL_AVG4;
    config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    config[1].cfg.gyr.range       = BMI2_GYR_RANGE_2000;
    config[1].cfg.gyr.odr         = BMI2_GYR_ODR_400HZ;
    config[1].cfg.gyr.bwp         = BMI2_GYR_NORMAL_MODE;
    config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    config[1].cfg.gyr.noise_perf  = BMI2_PERF_OPT_MODE;
 
    rslt = bmi270_set_sensor_config(config, 2, &dev);
    if (rslt != BMI2_OK) {
        printf("BMI270 set config failed: %d\n", rslt);
        return BMI_ERR;
    }
 
    struct bmi2_sens_config sig_cfg = {};
    sig_cfg.type = BMI2_SIG_MOTION;
 
    rslt = bmi270_get_sensor_config(&sig_cfg, 1, &dev);
    if (rslt == BMI2_OK) {
        sig_cfg.cfg.sig_motion.block_size = 0x0A;   // 10 samples = 200 ms at 50Hz
        bmi270_set_sensor_config(&sig_cfg, 1, &dev);
    }
 
    uint8_t sensor_list[3] = {BMI2_ACCEL, BMI2_GYRO, BMI2_SIG_MOTION};
    rslt = bmi270_sensor_enable(sensor_list, 3, &dev);
    if (rslt != BMI2_OK) {
        printf("BMI270 sensor enable failed: %d\n", rslt);
        return BMI_ERR;
    }
 

    return BMI_OK;
}

BMI_Status BMI270::calibration()
{
    printf("Starting BMI270 Calibration...\n");
    const int num_samples = 100;
    float sum_acc[3] = {0, 0, 0};
    float sum_gyr[3] = {0, 0, 0};
    int valid_samples = 0;

    while (valid_samples < num_samples) {
        uint8_t status = 0;

        if (bmi2_get_status(&status, &dev) == BMI2_OK) {
            if ((status & BMI2_DRDY_ACC) && (status & BMI2_DRDY_GYR)) {
                struct bmi2_sens_data raw = {};

                if (bmi2_get_sensor_data(&raw, &dev) == BMI2_OK) {
                    sum_acc[0] += raw.acc.x;
                    sum_acc[1] += raw.acc.y;
                    sum_acc[2] += raw.acc.z;

                    sum_gyr[0] += raw.gyr.x;
                    sum_gyr[1] += raw.gyr.y;
                    sum_gyr[2] += raw.gyr.z;

                    valid_samples++;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    accel_bias[0] = sum_acc[0] / num_samples;
    accel_bias[1] = sum_acc[1] / num_samples;
    accel_bias[2] = (sum_acc[2] / num_samples) - 16384.0f; // remove gravity

    gyro_bias[0] = sum_gyr[0] / num_samples;
    gyro_bias[1] = sum_gyr[1] / num_samples;
    gyro_bias[2] = sum_gyr[2] / num_samples;

    printf("Calibration OK\n");
    printf("Accel bias: %.2f %.2f %.2f\n", accel_bias[0], accel_bias[1], accel_bias[2]);
    printf("Gyro bias: %.2f %.2f %.2f\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    return BMI_OK;
}
 
BMI_data BMI270::getdata()
{
    BMI_data data = {};
 
    struct bmi2_sens_data raw = {};
    int8_t rslt = bmi2_get_sensor_data(&raw, &dev);
 
    if (rslt == BMI2_OK) {
        data.accx = (static_cast<float>(raw.acc.x) - accel_bias[0]) * ACCEL_RES_2G;
        data.accy = (static_cast<float>(raw.acc.y) - accel_bias[1]) * ACCEL_RES_2G;
        data.accz = (static_cast<float>(raw.acc.z) - accel_bias[2]) * ACCEL_RES_2G;
 
        data.gyrx = (static_cast<float>(raw.gyr.x) - gyro_bias[0]) * GYRO_RES_2000;
        data.gyry = (static_cast<float>(raw.gyr.y) - gyro_bias[1]) * GYRO_RES_2000;
        data.gyrz = (static_cast<float>(raw.gyr.z) - gyro_bias[2]) * GYRO_RES_2000;
    }
 
    uint16_t int_status = 0;
    if (bmi2_get_int_status(&int_status, &dev) == BMI2_OK) {
        data.ffall = ((int_status & BMI270_SIG_MOT_STATUS_MASK) != 0) ? 1 : 0;
    }
    return data;
}
 
BMI_Status BMI270::fault_detection()
{
    if (fault_override) return BMI_OK;
 
    uint8_t chip_id = 0;
    int8_t rslt = bmi2_get_regs(BMI2_CHIP_ID_ADDR, &chip_id, 1, &dev);
 
    if (rslt != BMI2_OK) {
        printf("BMI270 fault: SPI read error\n");
        return BMI_ERR;
    }
    if (chip_id != BMI270_CHIP_ID) {
        printf("BMI270 fault: wrong chip ID 0x%02X (expected 0x%02X)\n",
               chip_id, BMI270_CHIP_ID);
        return BMI_ERR;
    }
 
    uint8_t err_reg = 0;
    bmi2_get_err_reg_mask(&err_reg, &dev);
 
    if (err_reg & BMI2_FATAL_ERR_MASK) {
        printf("BMI270 fault: fatal error in ERR_REG\n");
        return BMI_ERR;
    }
 
    return BMI_OK;
}
