/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_log.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include <esp_matter.h>

#include <app_priv.h>

#define SCL_IO_PIN          GPIO_NUM_22
#define SDA_IO_PIN          GPIO_NUM_21
#define MASTER_FREQUENCY    100000
#define PORT_NUMBER         0
#define LENGTH              1


using namespace chip::app::Clusters;
using namespace esp_matter;

static const char *TAG = "app_driver";

i2c_master_dev_handle_t pfc_dev_handle = NULL;

static esp_err_t app_driver_update_gpio_value(gpio_num_t pin, bool value)
{
    esp_err_t err = ESP_OK;

    uint8_t buf[2] = {0, 0};
    u_int8_t len = 1;

    ESP_LOGI(TAG, "SW GPIO pin : %d set to %d", pin, value);

    err = i2c_master_receive(pfc_dev_handle, buf, len, 50);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get GPIO level");
    }
    ESP_LOGI(TAG, "read GPIO pin : 0x%x", buf[0]);

    buf[0] = buf[0] & (~(0x3<<(2*pin)));
	if(value == true){
		buf[0] |= (0x02 << (2 * pin));
	}
	else{
		buf[0] |= (0x01 << (2 * pin));
	}
    err = i2c_master_transmit(pfc_dev_handle, buf, len, 50);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set GPIO level");
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "EX-GPIO pin : %d set to 0x%x", pin, buf[0]);
    }

    vTaskDelay(100);

    buf[0] = 0;
    err = i2c_master_transmit(pfc_dev_handle, buf, len, 50);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set GPIO level");
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "EX-GPIO pin : %d set to 0x%x", pin, buf[0]);
    }

    return err;
}


esp_err_t app_driver_i2c_init()
{
    esp_err_t err = ESP_OK;

    if(pfc_dev_handle != NULL){
        return err;
    }

    ESP_LOGI(TAG, "app init extend gpio module");

    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = PORT_NUMBER,
        .sda_io_num = SDA_IO_PIN,
        .scl_io_num = SCL_IO_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;

    i2c_bus_config.flags.enable_internal_pullup = 1;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_device_config_t i2c_dev_conf = {
        .device_address = 0x20,
        .scl_speed_hz = MASTER_FREQUENCY,
    };
    err = i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &pfc_dev_handle);

    uint8_t cmd_buf[2] = {0};
    u_int8_t cmd_len = 1;
    i2c_master_transmit_receive(pfc_dev_handle, cmd_buf, cmd_len, cmd_buf, cmd_len, 100);

    ESP_LOGI(TAG, "i2c read: %x", cmd_buf[0]);

    return err;
}

esp_err_t app_driver_plugin_unit_init(const gpio_plug* plug)
{
    esp_err_t err = ESP_OK;

    // gpio_reset_pin(plug->GPIO_PIN_VALUE);

    // err = gpio_set_direction(plug->GPIO_PIN_VALUE, GPIO_MODE_OUTPUT);
    // if (err != ESP_OK) {
    //     ESP_LOGE(TAG, "Unable to set GPIO OUTPUT mode");
    //     return ESP_FAIL;
    // }

    // err = gpio_set_level(plug->GPIO_PIN_VALUE, 0);
    // if (err != ESP_OK) {
    //     ESP_LOGI(TAG, "Unable to set GPIO level");
    // }

    app_driver_i2c_init();

    return err;
}

// Return GPIO pin from plug-endpoint mapping list
gpio_num_t get_gpio(uint16_t endpoint_id)
{
    gpio_num_t gpio_pin = GPIO_NUM_NC;
    for (int i = 0; i < s_configure_plugs; i++) {
        if (s_plugin_unit_list[i].endpoint_id == endpoint_id) {
            gpio_pin = s_plugin_unit_list[i].plug;
        }
    }
    return gpio_pin;
}


esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val)
{
    esp_err_t err = ESP_OK;

    if (cluster_id == OnOff::Id) {
        if (attribute_id == OnOff::Attributes::OnOff::Id) {
            gpio_num_t gpio_pin = get_gpio(endpoint_id);
            if (gpio_pin != GPIO_NUM_NC) {
                err = app_driver_update_gpio_value(gpio_pin, val->val.b);
            } else {
                ESP_LOGE(TAG, "GPIO pin mapping for endpoint_id: %d not found", endpoint_id);
                return ESP_FAIL;
            }
        }
    }
    return err;
}


