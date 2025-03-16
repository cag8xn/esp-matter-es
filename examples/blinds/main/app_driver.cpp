/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


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

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <unistd.h>
#include <stdbool.h>
#include <cmath> // For std::round
//#include <esp_matter_cluster.h>
//#include <app/clusters/window_covering_server/window_covering_server.h>


using namespace chip::app::Clusters;
using namespace esp_matter;
using namespace esp_matter::cluster;

static const char *TAG = "app_driver";
extern uint16_t blinds_endpoint_id;
extern uint16_t blinds_top_endpoint_id;

// --- Configuration ---
#define STEP_PIN GPIO_NUM_18
#define DIR_PIN GPIO_NUM_19
#define ENABLE_PIN GPIO_NUM_5 // Optional, active low

#define STEP_PIN_TOP GPIO_NUM_21
#define DIR_PIN_TOP GPIO_NUM_22
#define ENABLE_PIN_TOP GPIO_NUM_35 // Optional, active low

#define TOP_LIMIT_SWITCH_PIN GPIO_NUM_25
#define BOTTOM_LIMIT_SWITCH_PIN GPIO_NUM_26

#define STEPS_PER_REVOLUTION 200 // Adjust based on your motor
#define MICROSTEPPING 1        // Adjust based on your DRV8825 settings
#define DEFAULT_STEP_DELAY_US 800 // Adjust for desired speed

#define direction_up 1
#define direction_down 0


void update_position_attribute(uint16_t endpoint_id, uint16_t new_position) {
    //using namespace chip::app::Clusters::WindowCovering;
    using namespace chip::app::Clusters::WindowCovering;

    esp_matter_attr_val_t attr_val;
    attr_val.type = ESP_MATTER_VAL_TYPE_UINT16;  // Set the value type
    attr_val.val.u16 = new_position;             // Store the new position

    esp_matter::attribute::update(
        endpoint_id,
        WindowCovering::Id,  // Pass the cluster ID (Window Covering cluster)
        Attributes::CurrentPositionLiftPercent100ths::Id,  // Attribute ID
        &attr_val  // Pass the struct instead of a raw pointer
    );
    //esp_matter::attribute::update(endpoint_id, WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id, &new_position, sizeof(new_position));
}


class BlindDriver {
private:
    bool is_calibrated = false;
    int max_steps = 0; // Steps from bottom limit to top limit
    volatile int current_position_steps = 0; // Current position in steps

    void configure_gpio();
    void step_motor(bool direction, int steps, int delay_us);
    bool is_top_limit_reached();
    bool is_bottom_limit_reached();
    int percent_to_steps(uint16_t percent);
    uint16_t steps_to_percent(int steps);

public:
    BlindDriver();
    void calibrate();
    void move_to_percent(uint16_t endpoint_id, uint16_t target_percent);
    uint16_t get_current_percent();
    void init();
};

BlindDriver::BlindDriver() {}

// --- GPIO Configuration ---
void BlindDriver::configure_gpio() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << STEP_PIN) | (1ULL << DIR_PIN);
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(STEP_PIN, 0);
    gpio_set_level(DIR_PIN, 0);

    // Configure optional ENABLE pin
    io_conf.pin_bit_mask = (1ULL << ENABLE_PIN);
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(ENABLE_PIN, 1); // Disable motor initially

    // Configure limit switch pins as inputs with pull-up
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pin_bit_mask = (1ULL << TOP_LIMIT_SWITCH_PIN) | (1ULL << BOTTOM_LIMIT_SWITCH_PIN);
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

// --- Limit Switch Readings ---
bool BlindDriver::is_top_limit_reached() {
    return gpio_get_level(TOP_LIMIT_SWITCH_PIN) == 0; // Assuming switch pulls low when active
}

bool BlindDriver::is_bottom_limit_reached() {
    return gpio_get_level(BOTTOM_LIMIT_SWITCH_PIN) == 0; // Assuming switch pulls low when active
}

// --- Stepper Motor Control ---
void BlindDriver::step_motor(bool direction, int steps, int delay_us) {
    if (!is_calibrated) {
        ESP_LOGW(TAG, "Motor not calibrated, cannot move.");
        return;
    }

    gpio_set_level(ENABLE_PIN, 0); // Enable the motor
    gpio_set_level(DIR_PIN, direction);

    int steps_moved = 0;
    while (steps_moved < steps) {
        // Safety check for limit switches
        if (is_top_limit_reached() || is_bottom_limit_reached()) {
            ESP_LOGW(TAG, "Limit switch triggered, stopping movement.");
            break;
        }

        gpio_set_level(STEP_PIN, 1);
        usleep(delay_us);
        gpio_set_level(STEP_PIN, 0);
        usleep(delay_us);

        steps_moved++;
        if (direction) {
            current_position_steps++;
        } else {
            current_position_steps--;
        }
    }

    gpio_set_level(ENABLE_PIN, 1); // Disable the motor
    ESP_LOGI(TAG, "Moved %d steps in direction %d. Current position: %d", steps_moved, direction, current_position_steps);
}

// --- Position Conversion ---
int BlindDriver::percent_to_steps(uint16_t percent) {
    if (!is_calibrated || max_steps == 0) {
        ESP_LOGW(TAG, "Cannot convert percentage to steps, motor not calibrated.");
        return current_position_steps; // Return current position if not calibrated
    }
    return static_cast<int>(std::round(static_cast<double>(percent) * max_steps / 10000.0));
}

uint16_t BlindDriver::steps_to_percent(int steps) {
    if (!is_calibrated || max_steps == 0) {
        ESP_LOGW(TAG, "Cannot convert steps to percentage, motor not calibrated.");
        return 0;
    }
    return static_cast<uint16_t>(std::round(static_cast<double>(steps) * 10000.0 / max_steps));
}

// --- Calibration Function ---
void BlindDriver::calibrate() {
    ESP_LOGI(TAG, "Starting blind calibration...");
    configure_gpio(); // Ensure GPIOs are configured

    // Move down until bottom limit is reached
    ESP_LOGI(TAG, "Moving down to find bottom limit...");
    gpio_set_level(ENABLE_PIN, 0);
    gpio_set_level(DIR_PIN, direction_down); // Set direction down
    while (!is_bottom_limit_reached()) {
        gpio_set_level(STEP_PIN, 1);
        usleep(DEFAULT_STEP_DELAY_US);
        gpio_set_level(STEP_PIN, 0);
        usleep(DEFAULT_STEP_DELAY_US);
        //vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to allow switch to settle
    }
    gpio_set_level(ENABLE_PIN, 1);
    current_position_steps = 0; // Reset position to 0 at bottom limit
    ESP_LOGI(TAG, "Bottom limit reached.");
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait a bit

    // Move up until top limit is reached
    ESP_LOGI(TAG, "Moving up to find top limit...");
    gpio_set_level(ENABLE_PIN, 0);
    gpio_set_level(DIR_PIN, direction_up); // Set direction up
    int steps_counted = 0;
    while (!is_top_limit_reached()) {
        gpio_set_level(STEP_PIN, 1);
        usleep(DEFAULT_STEP_DELAY_US);
        gpio_set_level(STEP_PIN, 0);
        usleep(DEFAULT_STEP_DELAY_US);
        steps_counted++;
        //vTaskDelay(pdMS_TO_TICKS(10)); // Small delay
    }
    gpio_set_level(ENABLE_PIN, 1);
    max_steps = steps_counted;
    is_calibrated = true;
    ESP_LOGI(TAG, "Top limit reached. Calibration complete. Max steps: %d", max_steps);
}

// --- Move to Target Position Function ---
void BlindDriver::move_to_percent(uint16_t endpoint_id, uint16_t target_percent) {
    if (!is_calibrated) {
        ESP_LOGW(TAG, "Cannot move to target, motor not calibrated.");
        return;
    }

    int target_steps = percent_to_steps(target_percent);
    ESP_LOGI(TAG, "Moving to target percentage: %d (steps: %d). Current position: %d",
             target_percent, target_steps, current_position_steps);

    int steps_to_move = target_steps - current_position_steps;
    bool direction = (steps_to_move > 0);
    int abs_steps_to_move = std::abs(steps_to_move);

    if (abs_steps_to_move > 0) {
        step_motor(direction, abs_steps_to_move, DEFAULT_STEP_DELAY_US);
        // ELIA: attempt to fix the homekit visualization
        update_position_attribute(endpoint_id, target_percent);
    } else {
        ESP_LOGI(TAG, "Target position is the same as current position.");
    }
}

// --- Get Current Position ---
uint16_t BlindDriver::get_current_percent() {
    return steps_to_percent(current_position_steps);
}

// --- Driver Initialization ---
void BlindDriver::init() {
    configure_gpio();
    ESP_LOGI(TAG, "Blind driver initialized.");
    // Calibration will be called separately after boot, as requested.
}

// --- Global Instance of the Driver ---
static BlindDriver blind_driver;

void blind_driver_calibrate() {
    blind_driver.calibrate();
}

void blind_driver_move_to_percent(uint16_t endpoint_id, uint16_t target_percent) {
    blind_driver.move_to_percent(endpoint_id, target_percent);
}

uint16_t blind_driver_get_current_percent() {
    return blind_driver.get_current_percent();
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
        if (cluster_id == WindowCovering::Id) {
            ESP_LOGI(TAG, "8===============================D Received cluster message");
            if (attribute_id == WindowCovering::Attributes::TargetPositionLiftPercent100ths::Id) {
                // Adjust the lift percentage if needed
                ESP_LOGI(TAG, "8===============================D Received attribute update: TargetPositionLiftPercent100ths");
                uint16_t target_percent;
                // Access the uint16_t value from the esp_matter_attr_val_t structure
                target_percent = val->val.u16;
                blind_driver_move_to_percent(endpoint_id, target_percent); // Corrected call
                //blind_driver_calibrate();
            } 
        }
    }
    else if (endpoint_id == blinds_top_endpoint_id){
        ESP_LOGI(TAG, "TOP!!  8===============================D Received endpoint message");
        if (cluster_id == WindowCovering::Id) {
            ESP_LOGI(TAG, "TOP!!  8===============================D Received cluster message");
            if (attribute_id == WindowCovering::Attributes::TargetPositionLiftPercent100ths::Id) {
                // Adjust the lift percentage if needed
                ESP_LOGI(TAG, "TOP!!  8===============================D Received attribute update: TargetPositionLiftPercent100ths");
                uint16_t target_percent;
                // Access the uint16_t value from the esp_matter_attr_val_t structure
                target_percent = val->val.u16;
                //blind_driver_move_to_percent(target_percent); // Corrected call
                //blind_driver_calibrate();
            } 
        }
    }
    return err;
}


esp_err_t app_driver_blinds_init(uint16_t endpoint_id)
{
    ESP_LOGI(TAG, "8===============================D INIT !!!!!");
    blind_driver.init();
    vTaskDelay(pdMS_TO_TICKS(2000));
    //usleep(2000);
    blind_driver_calibrate();
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