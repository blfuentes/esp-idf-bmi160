#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <esp_log.h>
#include "bmi160_defs.h"
#include "bmi160.h"

#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_21

static const char *TAG = "BMI160_SPI";
spi_device_handle_t spi;  // Global handle for the SPI device

// Function prototypes for SPI read/write
int8_t bmi160_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t bmi160_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void delay_ms(uint32_t ms);

int8_t bmi160_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    uint8_t tx_buffer[len + 1];
    uint8_t rx_buffer[len + 1];
    tx_buffer[0] = (reg_addr | 0x80); // Set MSB for read operation

    spi_transaction_t t = {
        .length = static_cast<size_t>((len + 1) * 8), // Length in bits
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer
    };

    esp_err_t ret = spi_device_transmit(spi, &t);
    if (ret == ESP_OK) {
        memcpy(data, &rx_buffer[1], len); // Copy data to the output buffer
        return BMI160_OK;
    } else {
        ESP_LOGE(TAG, "SPI Read failed");
        return BMI160_E_COM_FAIL;
    }
}

// SPI write function
int8_t bmi160_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    uint8_t tx_buffer[len+1];
    tx_buffer[0] = reg_addr & ~0x80; // Clear MSB for write operation
    memcpy(&tx_buffer[1], data, len);

    spi_transaction_t t = {
        .length = static_cast<size_t>((len + 1) * 8), // Length in bits
        .tx_buffer = tx_buffer
    };
    spi_device_transmit(spi, &t);
    return BMI160_OK;
}

// Delay function for BMI160 API
void delay_ms(uint32_t ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"

struct transformed_data_t {
    int16_t x;
    int16_t y;
    int16_t z; 
};

transformed_data_t transform_accel_data(bmi160_sensor_data rawdata) {
    transformed_data_t data;
    data.x = rawdata.x / (32768 / 16);
    data.y = rawdata.y / (32768 / 16);
    data.z = rawdata.z / (32768 / 16);

    return data;
}

transformed_data_t transform_gyro_data(bmi160_sensor_data rawdata) {
    transformed_data_t data;
    data.x = rawdata.x / (32768 / 2000);
    data.y = rawdata.y / (32768 / 2000);
    data.z = rawdata.z / (32768 / 2000);

    return data;
}
#pragma GCC diagnostic pop


extern "C" void app_main();


void app_main(void) {
    esp_err_t ret;

    gpio_reset_pin(PIN_NUM_MISO); // MISO
    gpio_reset_pin(PIN_NUM_MOSI); // MOSI
    gpio_reset_pin(PIN_NUM_CLK); // SCLK
    gpio_reset_pin(PIN_NUM_CS);  // CS

    // Initialize SPI bus
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(spi_bus_config_t));
    // buscfg.intr_flags = 0;
    buscfg.miso_io_num = PIN_NUM_MISO;
    buscfg.mosi_io_num = PIN_NUM_MOSI;
    buscfg.sclk_io_num = PIN_NUM_CLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 4092;
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);

    // Configure SPI device
    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(spi_device_interface_config_t));
    devcfg.clock_speed_hz = 5*1000*1000;           // Clock out at 5 MHz
    devcfg.mode = 0;                               // SPI mode 0
    devcfg.spics_io_num = PIN_NUM_CS;              // CS pin
    devcfg.queue_size = 7;                         // We want to be able to queue 7 transactions at a time
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    // Initialize BMI160
    struct bmi160_dev sensor;
    sensor.id = PIN_NUM_CS;  // any value is ok...
    sensor.intf = BMI160_SPI_INTF;
    sensor.read = bmi160_spi_read;
    sensor.write = bmi160_spi_write;
    sensor.delay_ms = delay_ms;

    int8_t rslt = bmi160_init(&sensor);
    if (rslt != BMI160_OK) {
        ESP_LOGE(TAG, "BMI160 Initialization failed with code: %d", rslt);
        return;
    }

    ESP_LOGI(TAG, "BMI160 sensor initialized successfully");

    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    sensor.accel_cfg.range = BMI160_ACCEL_RANGE_16G;
    sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    sensor.accel_cfg.power = BMI160_ACCEL_LOWPOWER_MODE;

    sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    rslt = bmi160_set_sens_conf(&sensor);
    if (rslt != BMI160_OK) {
        ESP_LOGE(TAG, "Failed to configure sensor");
        return;
    }

    // Set power mode
    rslt = bmi160_set_power_mode(&sensor);
    if (rslt != BMI160_OK) {
        ESP_LOGE(TAG, "Failed to set power mode");
        return;
    }

    // Read sensor data
    struct bmi160_sensor_data accel;
    struct bmi160_sensor_data gyro;
    struct transformed_data_t acceldata;
    struct transformed_data_t gyrodata;

    while (1) {

    // uint8_t reg_data;
    // bmi160_spi_read(sensor.id, BMI160_ACCEL_X_LSB_ADDR, &reg_data, 1); // Example: Read X-axis LSB
    // ESP_LOGI(TAG, "Accel X LSB: 0x%02X", reg_data);

        rslt = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accel, &gyro, &sensor);

        if (rslt == BMI160_OK) {
            acceldata = transform_accel_data(accel);
            gyrodata = transform_gyro_data(gyro);

            ESP_LOGI(TAG, "Accel X: %d mg, Y: %d mg, Z: %d mg. Sensor: %ld", 
                     acceldata.x, acceldata.y, acceldata.z, accel.sensortime);
            ESP_LOGI(TAG, "Gyro X: %d dps, Y: %d dps, Z: %d dps. Sensor: %ld", 
                     gyrodata.x, gyrodata.y, gyrodata.z, gyro.sensortime);
        } else {
            ESP_LOGE(TAG, "Failed to read sensor data");
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
