/*
 bmp180.c - BMP180 pressure sensor driver for ESP32

 This file is part of the ESP32 Everest Run project
 https://github.com/krzychb/esp32-everest-run

 Copyright (c) 2016 Krzysztof Budzynski <krzychb@gazeta.pl>
 This work is licensed under the Apache License, Version 2.0, January 2004
 See the file LICENSE for details.
 */

#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/i2c.h"
#include "esp_log.h"

#include "bmp180.h"

static const char* TAG = "BMP180 I2C Driver";

#define ACK_CHECK_EN    0x1     // I2C master will check ack from slave
#define ACK_CHECK_DIS   0x0     // I2C master will not check ack from slave
#define ACK_VAL         0x0     // I2C ack value
#define NACK_VAL        0x1     // I2C nack value

#define BMP180_ADDRESS 0x77     // I2C address of BMP180

#define BMP180_ULTRA_LOW_POWER  0
#define BMP180_STANDARD         1
#define BMP180_HIGH_RES         2
#define BMP180_ULTRA_HIGH_RES   3

#define BMP180_CAL_AC1          0xAA  // Calibration data (16 bits)
#define BMP180_CAL_AC2          0xAC  // Calibration data (16 bits)
#define BMP180_CAL_AC3          0xAE  // Calibration data (16 bits)
#define BMP180_CAL_AC4          0xB0  // Calibration data (16 bits)
#define BMP180_CAL_AC5          0xB2  // Calibration data (16 bits)
#define BMP180_CAL_AC6          0xB4  // Calibration data (16 bits)
#define BMP180_CAL_B1           0xB6  // Calibration data (16 bits)
#define BMP180_CAL_B2           0xB8  // Calibration data (16 bits)
#define BMP180_CAL_MB           0xBA  // Calibration data (16 bits)
#define BMP180_CAL_MC           0xBC  // Calibration data (16 bits)
#define BMP180_CAL_MD           0xBE  // Calibration data (16 bits)

#define BMP180_CONTROL             0xF4  // Control register
#define BMP180_DATA_TO_READ        0xF6  // Read results here
#define BMP180_READ_TEMP_CMD       0x2E  // Request temperature measurement
#define BMP180_READ_PRESSURE_CMD   0x34  // Request pressure measurement

// Calibration parameters
static int16_t ac1;
static int16_t ac2;
static int16_t ac3;
static uint16_t ac4;
static uint16_t ac5;
static uint16_t ac6;
static int16_t b1;
static int16_t b2;
static int16_t mb;
static int16_t mc;
static int16_t md;
static uint8_t oversampling = BMP180_ULTRA_HIGH_RES;


static esp_err_t bmp180_master_write_slave(i2c_port_t i2c_num, uint8_t* data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( BMP180_ADDRESS  << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


static esp_err_t bmp180_write_reg(i2c_port_t i2c_num, uint8_t reg, uint8_t cmd)
{
    uint8_t data_wr[] = {reg, cmd};
    esp_err_t err = bmp180_master_write_slave(i2c_num, data_wr, 2);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Write [0x%02x] = 0x%02x failed, err = %d", reg, cmd, err);
    }
    return err;
}


static esp_err_t bmp180_master_read_slave(i2c_port_t i2c_num, uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( BMP180_ADDRESS << 1 ) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


static esp_err_t bmp180_read_int16(i2c_port_t i2c_num, uint8_t reg, int16_t* value)
{
    esp_err_t err = bmp180_master_write_slave(i2c_num, &reg, 1);
    if (err == ESP_OK) {
        uint8_t data_rd[2] = {0};
        err = bmp180_master_read_slave(i2c_num, data_rd, 2);
        if (err == ESP_OK) {
            *value = (int16_t) ((data_rd[0] << 8) | data_rd[1]);
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read [0x%02x] int16 failed, err = %d", reg, err);
    }
    return err;
}


static esp_err_t bmp180_read_uint16(i2c_port_t i2c_num, uint8_t reg, uint16_t* value)
{
    esp_err_t err = bmp180_master_write_slave(i2c_num, &reg, 1);
    if (err == ESP_OK) {
        uint8_t data_rd[2] = {0};
        err = bmp180_master_read_slave(i2c_num, data_rd, 2);
        if (err == ESP_OK) {
            *value = (uint16_t) ((data_rd[0] << 8) | data_rd[1]);
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read [0x%02x] uint16 failed, err = %d", reg, err);
    }
    return err;
}


static esp_err_t bmp180_read_uint32(i2c_port_t i2c_num, uint8_t reg, uint32_t* value)
{
    esp_err_t err = bmp180_master_write_slave(i2c_num, &reg, 1);
    if (err == ESP_OK) {
        uint8_t data_rd[3] = {0};
        err = bmp180_master_read_slave(i2c_num, data_rd, 3);
        if (err == ESP_OK) {
            *value = (uint32_t) ((data_rd[0] << 16) | (data_rd[1] << 8) | data_rd[2]);
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read [0x%02x] uint16 failed, err = %d", reg, err);
    }
    return err;
}


static esp_err_t bmp180_read_uncompensated_temperature(int16_t* temp)
{
    esp_err_t err = bmp180_write_reg(I2C_NUM_0, BMP180_CONTROL, BMP180_READ_TEMP_CMD);
    if (err == ESP_OK) {
        vTaskDelay(5 / portTICK_RATE_MS);
        err = bmp180_read_int16(I2C_NUM_0, BMP180_DATA_TO_READ, temp);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read uncompensated temperature failed, err = %d", err);
    }
    return err;
}


static esp_err_t bmp180_calculate_b5(int32_t* b5)
{
    int16_t ut;
    int32_t x1, x2;

    esp_err_t err = bmp180_read_uncompensated_temperature(&ut);
    if (err == ESP_OK) {
        x1 = ((ut - (int32_t) ac6) * (int32_t) ac5) >> 15;
        x2 = ((int32_t) mc << 11) / (x1 + md);
        *b5 = x1 + x2;
    } else {
        ESP_LOGE(TAG, "Calculate b5 failed, err = %d", err);
    }
    return err;
}

static uint32_t bmp180_read_uncompensated_pressure(uint32_t* up)
{
    esp_err_t err = bmp180_write_reg(I2C_NUM_0, BMP180_CONTROL, BMP180_READ_PRESSURE_CMD + (oversampling << 6));
    if (err == ESP_OK) {
        uint8_t wait_time_ms = 2 + (3 << oversampling);
        vTaskDelay(wait_time_ms / portTICK_RATE_MS);

        err = bmp180_read_uint32(I2C_NUM_0, BMP180_DATA_TO_READ, up);
        if (err == ESP_OK) {
            *up >>= (8 - oversampling);
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read uncompensated pressure failed, err = %d", err);
    }
    return err;
}


esp_err_t bmp180_read_temperature(float* temperature)
{
    int32_t b5;

    esp_err_t err = bmp180_calculate_b5(&b5);
    if (err == ESP_OK) {
        *temperature = ((b5 + 8) >> 4) / 10.0;
    } else {
        ESP_LOGE(TAG, "Read temperature failed, err = %d", err);
    }
    return err;
}


esp_err_t bmp180_read_pressure(uint32_t* pressure)
{
    int32_t b3, b5, b6, x1, x2, x3, p;
    uint32_t up, b4, b7;
    esp_err_t err;

    err = bmp180_calculate_b5(&b5);
    if (err == ESP_OK) {
        b6 = b5 - 4000;
        x1 = (b2 * (b6 * b6) >> 12) >> 11;
        x2 = (ac2 * b6) >> 11;
        x3 = x1 + x2;
        b3 = (((((int32_t)ac1) * 4 + x3) << oversampling) + 2) >> 2;

        x1 = (ac3 * b6) >> 13;
        x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
        x3 = ((x1 + x2) + 2) >> 2;
        b4 = (ac4 * (uint32_t)(x3 + 32768)) >> 15;

        err  = bmp180_read_uncompensated_pressure(&up);
        if (err == ESP_OK) {
            b7 = ((uint32_t)(up - b3) * (50000 >> oversampling));
            if (b7 < 0x80000000) {
                p = (b7 << 1) / b4;
            } else {
                p = (b7 / b4) << 1;
            }

            x1 = (p >> 8) * (p >> 8);
            x1 = (x1 * 3038) >> 16;
            x2 = (-7357 * p) >> 16;
            p += (x1 + x2 + 3791) >> 4;
            *pressure = p;
        }
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Pressure compensation failed, err = %d", err);
    }

    return err;
}


esp_err_t bmp180_read_altitude(uint32_t reference_pressure, float* altitude)
{
    uint32_t absolute_pressure;
    esp_err_t err = bmp180_read_pressure(&absolute_pressure);
    if (err == ESP_OK) {
        *altitude =  44330 * (1.0 - powf(absolute_pressure / (float) reference_pressure, 0.190295));
    } else {
        ESP_LOGE(TAG, "Read altitude failed, err = %d", err);
    }
    return err;
}


esp_err_t bmp180_init(int pin_sda, int pin_scl)
{
    esp_err_t err;

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = pin_sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = pin_scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000 ;

    err = i2c_param_config(I2C_NUM_0, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver configuration failed with error = %d", err);
        return ESP_ERR_BMP180_NOT_DETECTED;
    }
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver installation failed with error = %d", err);
        return ESP_ERR_BMP180_NOT_DETECTED;
    }
    ESP_LOGI(TAG, "I2C master driver has been installed.");

    uint8_t reg = 0x00;
    err = bmp180_master_write_slave(I2C_NUM_0, &reg, 1);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BMP180 sensor not found at 0x%02x", BMP180_ADDRESS);
        return ESP_ERR_BMP180_NOT_DETECTED;
    }

    ESP_LOGI(TAG, "BMP180 sensor found at 0x%02x", BMP180_ADDRESS);
    bmp180_read_int16(I2C_NUM_0, BMP180_CAL_AC1, &ac1);
    bmp180_read_int16(I2C_NUM_0, BMP180_CAL_AC2, &ac2);
    bmp180_read_int16(I2C_NUM_0, BMP180_CAL_AC3, &ac3);
    bmp180_read_uint16(I2C_NUM_0, BMP180_CAL_AC4, &ac4);
    bmp180_read_uint16(I2C_NUM_0, BMP180_CAL_AC5, &ac5);
    bmp180_read_uint16(I2C_NUM_0, BMP180_CAL_AC6, &ac6);
    bmp180_read_int16(I2C_NUM_0, BMP180_CAL_B1, &b1);
    bmp180_read_int16(I2C_NUM_0, BMP180_CAL_B2, &b2);
    bmp180_read_int16(I2C_NUM_0, BMP180_CAL_MB, &mb);
    bmp180_read_int16(I2C_NUM_0, BMP180_CAL_MC, &mc);
    bmp180_read_int16(I2C_NUM_0, BMP180_CAL_MD, &md);

    return ESP_OK;
}
