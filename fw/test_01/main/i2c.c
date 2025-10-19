#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/i2c.h"

#include "i2c.h"

static const char *TAG = "i2c";

#define I2C_MASTER_SCL_IO           15                          /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           16                          /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                           /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000                      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       50

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void pcf8574_write(uint8_t addr,uint8_t value)
{
    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM, 0x38 + addr, &value, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
}

uint8_t pcf8574_read(uint8_t addr)
{
    uint8_t data = 0;
    ESP_ERROR_CHECK(i2c_master_read_from_device(I2C_MASTER_NUM, 0x38 + addr, &data, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
    return data;
}

void i2c_set_gpio(uint16_t pin,uint8_t value)
{
    uint8_t v = pcf8574_read((pin>>8)&0xff);
    v =~ (1<<(pin&0xff));
    v |= (value<<(pin&0xff));
    pcf8574_write((pin>>8)&0xff,v);
}


void i2c_init()
{
    i2c_master_init();
    pcf8574_write(0,0x00);
    i2c_set_gpio(I2CIO_PCF8574_BEEP, 1);
    i2c_set_gpio(I2CIO_PCF8574_PON, 1);
    

    i2c_set_gpio(I2CIO_PCF8574_CAM_RST, 0);
    i2c_set_gpio(I2CIO_PCF8574_CAM_PWDN, 0);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    i2c_set_gpio(I2CIO_PCF8574_CAM_PWDN, 1);
    i2c_set_gpio(I2CIO_PCF8574_CAM_RST, 1);

    vTaskDelay(100 / portTICK_PERIOD_MS);
    i2c_set_gpio(I2CIO_PCF8574_BEEP, 0);
}

void i2c_handle()
{
    //ESP_LOGI(TAG, "%02x",pcf8574_read(0));    
}

void i2c_exit()
{
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}

