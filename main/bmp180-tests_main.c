/* BMP180 Tests

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "bmp180.h"
#include "bmp180_twi.h"


static const char *BMP180_TWI_LOG_TAG = "BMP180 TWI Read";
static const char *BMP180_I2C_LOG_TAG = "BMP180 I2C Read";

/* Compensate altitude measurement
   using current reference pressure, preferably at the sea level,
   obtained from weather station on internet
   Assume normal air pressure at sea level of 101325 Pa
   in case weather station is not available.
 */
#define REFERENCE_PRESSURE 101325l

/* Define pins to connect I2C pressure sensor
*/
#define TWI_PIN_SDA 25
#define TWI_PIN_SCL 27

#define I2C_PIN_SDA 22
#define I2C_PIN_SCL 23


void bmp180_twi_task(void *pvParameter)
{
    while(1) {
        uint32_t pressure = bmp180_twi_read_pressure();
        float altitude = bmp180_twi_read_altitude(REFERENCE_PRESSURE);
        float temperature = bmp180_twi_read_temperature();
        ESP_LOGI(BMP180_TWI_LOG_TAG, "Pressure %d Pa, Altitude %.1f m, Temperature : %.1f oC", pressure, altitude, temperature);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


void bmp180_i2c_task(void *pvParameter)
{
    while(1) {
        esp_err_t err;
        uint32_t pressure;
        float altitude;
        float temperature;

        err = bmp180_read_pressure(&pressure);
        if (err != ESP_OK) {
            ESP_LOGE(BMP180_I2C_LOG_TAG, "Reading of pressure from BMP180 failed, err = %d", err);
        }
        err = bmp180_read_altitude(REFERENCE_PRESSURE, &altitude);
        if (err != ESP_OK) {
            ESP_LOGE(BMP180_I2C_LOG_TAG, "Reading of altitude from BMP180 failed, err = %d", err);
        }
        err = bmp180_read_temperature(&temperature);
        if (err != ESP_OK) {
            ESP_LOGE(BMP180_I2C_LOG_TAG, "Reading of temperature from BMP180 failed, err = %d", err);
        }
        ESP_LOGI(BMP180_I2C_LOG_TAG, "Pressure %d Pa, Altitude %.1f m, Temperature : %.1f oC", pressure, altitude, temperature);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


void app_main()
{
    esp_err_t err;

    ESP_LOGI(BMP180_I2C_LOG_TAG, "Main application is starting...");

    err = bmp180_twi_init(TWI_PIN_SDA, TWI_PIN_SCL);
    if(err == ESP_OK){
        xTaskCreate(&bmp180_twi_task, "bmp180_twi_task", 1024*4, NULL, 5, NULL);
    } else {
        ESP_LOGE(BMP180_TWI_LOG_TAG, "BMP180 init failed with error = %d", err);
    }

    err = bmp180_init(I2C_PIN_SDA, I2C_PIN_SCL);
    if(err == ESP_OK){
        xTaskCreate(&bmp180_i2c_task, "bmp180_i2c_task", 1024*4, NULL, 5, NULL);
    } else {
        ESP_LOGE(BMP180_I2C_LOG_TAG, "BMP180 init failed with error = %d", err);
    }
}

