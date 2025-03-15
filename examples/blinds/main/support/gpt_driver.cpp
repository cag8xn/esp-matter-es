/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_log.h>
#include <stdlib.h>
#include <string.h>
#include <device.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <app_reset.h>

#include <esp_matter.h>
#include "bsp/esp-bsp.h"

#include <app_priv.h>

#include <driver/gpio.h>
#include <driver/mcpwm_prelude.h>
#include "drv8825.h"   // Assume a DRV8825 driver abstraction

using namespace chip::app::Clusters;
using namespace esp_matter;
using namespace esp_matter::cluster;

static const char *TAG = "app_driver";
extern uint16_t blinds_endpoint_id;

#define LIMIT_SWITCH_TOP GPIO_NUM_25
#define LIMIT_SWITCH_BOTTOM GPIO_NUM_26
#define STEPPER_STEP GPIO_NUM_18
#define STEPPER_DIR GPIO_NUM_19
#define STEPPER_ENABLE GPIO_NUM_5


static int min_position = 0;
static int max_position = 0;
static int current_position = 0;

void init_limit_switches() {
    ESP_LOGI(TAG, "8============================================D - - Initializing limit switches");
    
    gpio_set_direction(LIMIT_SWITCH_TOP, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LIMIT_SWITCH_TOP, GPIO_PULLUP_ONLY);
    
    gpio_set_direction(LIMIT_SWITCH_BOTTOM, GPIO_MODE_INPUT);
    gpio_set_pull_mode(LIMIT_SWITCH_BOTTOM, GPIO_PULLUP_ONLY);
}

void calibrate_blinds() {
    ESP_LOGI(TAG, "Starting calibration");
    drv8825_enable(STEPPER_ENABLE, true);
    drv8825_set_direction(STEPPER_DIR, false);

    while (!gpio_get_level(LIMIT_SWITCH_BOTTOM)) {
        drv8825_step(STEPPER_STEP);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    min_position = 0;
    current_position = 0;

    drv8825_set_direction(STEPPER_DIR, true);
    while (!gpio_get_level(LIMIT_SWITCH_TOP)) {
        drv8825_step(STEPPER_STEP);
        vTaskDelay(pdMS_TO_TICKS(2));
        max_position++;
    }
    ESP_LOGI(TAG, "Calibration complete: min=%d, max=%d", min_position, max_position);
}

void move_to_position(int target_percent) {
    int target_position = min_position + (max_position - min_position) * target_percent / 10000;
    ESP_LOGI(TAG, "Moving to position: %d", target_position);
    
    if (target_position > current_position) {
        drv8825_set_direction(STEPPER_DIR, true);
        while (current_position < target_position && !gpio_get_level(LIMIT_SWITCH_TOP)) {
            drv8825_step(STEPPER_STEP);
            current_position++;
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    } else {
        drv8825_set_direction(STEPPER_DIR, false);
        while (current_position > target_position && !gpio_get_level(LIMIT_SWITCH_BOTTOM)) {
            drv8825_step(STEPPER_STEP);
            current_position--;
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
    ESP_LOGI(TAG, "Movement complete: current=%d", current_position);
}



// ELIA: Following code TOGGLES the OnOff attribute.
static void toggle_OnOff_attribute() {
    uint16_t endpoint_id = blinds_endpoint_id;
    uint32_t cluster_id = OnOff::Id;
    uint32_t attribute_id = OnOff::Attributes::OnOff::Id;

    node_t *node = node::get();
    endpoint_t *endpoint = endpoint::get(node, endpoint_id);
    cluster_t *cluster = cluster::get(endpoint, cluster_id);
    attribute_t *attribute = attribute::get(cluster, attribute_id);

    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    attribute::get_val(attribute, &val);
    val.val.b = !val.val.b;
    attribute::update(endpoint_id, cluster_id, attribute_id, &val);
}

static void launch(void *pvParameters) {
    int i = 0;
}

static esp_err_t app_driver_blinds_set_power(led_indicator_handle_t handle, esp_matter_attr_val_t *val)
{
    if (val->val.b) {
        ESP_LOGI(TAG, "================================== Catapult power ON signal received: %d", val->val.b);
        xTaskCreate(&launch,         // Function to run
                "launch_task",   // Task name
                4096,                 // Stack size (in words, not bytes)
                NULL,                 // No parameter to pass to the task
                5,                    // Priority (higher number means higher priority)
                NULL);                // Task handle (can be NULL if not needed)

        //launch();

        ESP_LOGI(TAG, "ELIA ================================== Attempting to turn OFF the OnOff attribute");
        toggle_OnOff_attribute();
    }
    return ESP_OK;
}


static void app_driver_button_toggle_cb(void *arg, void *data)
{
    ESP_LOGI(TAG, "Toggle button pressed");
    toggle_OnOff_attribute();
}

esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val)
{
    ESP_LOGI(TAG, "8===============================D In the driver");
    ESP_LOGI(TAG, "8===============================D : endpoint_id = %d, cluster_id = %lu, attribute_id = %lu", endpoint_id, cluster_id, attribute_id);
    ESP_LOGI(TAG, "8===============================D : endpoint_id = %d, blinds_endpoint_id = %d", endpoint_id, blinds_endpoint_id);
    esp_err_t err = ESP_OK;
    if (endpoint_id == blinds_endpoint_id) {
        ESP_LOGI(TAG, "8===============================D Received endpoint message");
        led_indicator_handle_t handle = (led_indicator_handle_t)driver_handle;
        if (cluster_id == WindowCovering::Id) {
            ESP_LOGI(TAG, "8===============================D Received cluster message");
            if (attribute_id == WindowCovering::Attributes::CurrentPositionLift::Id) {
                // Set the blinds position based on the attribute value
                // err = app_driver_blinds_set_position(handle, val);
                ESP_LOGI("CurrentPosition", "Received attribute update: endpoint_id = %d, cluster_id = %lu, attribute_id = %lu", endpoint_id, cluster_id, attribute_id);
            } else if (attribute_id == WindowCovering::Attributes::CurrentPositionLiftPercentage::Id) {
                // Set the target position for the blinds
                // err = app_driver_blinds_set_target_position(handle, val);
                ESP_LOGI("CurrentPositionLiftPercentage", "Received attribute update: endpoint_id = %d, cluster_id = %lu, attribute_id = %lu", endpoint_id, cluster_id, attribute_id);
            } else if (attribute_id == WindowCovering::Attributes::TargetPositionLiftPercent100ths::Id) {
                // Adjust the lift percentage if needed
                // err = app_driver_blinds_set_lift_percentage(handle, val);
                ESP_LOGI(TAG, "8===============================D Received attribute update: TargetPositionLiftPercent100ths");
                calibrate_blinds();
            } else if (attribute_id == WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id) {
                // Set the target lift percentage
                // err = app_driver_blinds_set_target_lift_percentage(handle, val);
                ESP_LOGI("CurrentPositionLiftPercent100ths", "Received attribute update: endpoint_id = %d, cluster_id = %lu, attribute_id = %lu", endpoint_id, cluster_id, attribute_id);
            }
        }
    }
    return err;
}


esp_err_t app_driver_blinds_init(uint16_t endpoint_id)
{
    init_limit_switches();
    return ESP_OK;
}

app_driver_handle_t app_driver_button_init()
{
    /* Initialize button */
    button_config_t config = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = 17,
            .active_level = 0,
        }
    };
    button_handle_t handle = iot_button_create(&config);
    iot_button_register_cb(handle, BUTTON_PRESS_DOWN, app_driver_button_toggle_cb, NULL);
    gpio_set_pull_mode((gpio_num_t)17, GPIO_PULLUP_ONLY);
    
    return (app_driver_handle_t)handle;
}

app_driver_handle_t app_driver_reset_button_init()
{
    /* Initialize button */
    button_config_t config = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = 16,
            .active_level = 0,
        }
    };
    button_handle_t handle = iot_button_create(&config);
    app_reset_button_register(handle);

    // Enable internal pull-up resistor for GPIO 17
    gpio_set_pull_mode((gpio_num_t)16, GPIO_PULLUP_ONLY);
    
    return (app_driver_handle_t)handle;
}