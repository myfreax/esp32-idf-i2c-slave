#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
static const char *TAG = "i2c-slave";
#define I2C_SLAVE_SCL_IO 22
#define I2C_SLAVE_SDA_IO 21
#define I2C_SLAVE_RX_BUF_LEN 255
#define ESP_SLAVE_ADDR 0x0A
#define I2C_PORT 0

static esp_err_t i2c_init(void) {
  i2c_config_t config = {
      .sda_io_num = I2C_SLAVE_SDA_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_io_num = I2C_SLAVE_SCL_IO,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .mode = I2C_MODE_SLAVE,
      .slave.addr_10bit_en = 0,  // use 7 bit address, disable 10 bit address
      .slave.slave_addr = ESP_SLAVE_ADDR,  // slave 7 bit addr of your project
      .clk_flags = 0,
  };
  esp_err_t err = i2c_param_config(I2C_PORT, &config);
  if (err != ESP_OK) {
    return err;
  }
  return i2c_driver_install(I2C_PORT, config.mode, I2C_SLAVE_RX_BUF_LEN, 255,
                            0);
}

void app_main(void) {
  uint8_t message[] = "LED_ENABLED";
  uint8_t data[I2C_SLAVE_RX_BUF_LEN] = {0};
  ESP_ERROR_CHECK(i2c_init());
  ESP_LOGI(TAG, "I2C Slave Initialized Successfully");
  int read_buffer_len;
  while (1) {
    read_buffer_len = i2c_slave_read_buffer(
        I2C_PORT, data, I2C_SLAVE_RX_BUF_LEN, 1000 / portTICK_PERIOD_MS);
    // Send data to master when salve reviced data from master
    if (read_buffer_len > 0) {
      ESP_LOGI(TAG, "Data Recived = %s", data);
      i2c_slave_write_buffer(I2C_PORT, message, sizeof(message),
                             1000 / portTICK_PERIOD_MS);
      memset(data, 0, I2C_SLAVE_RX_BUF_LEN);
    }
  }
}