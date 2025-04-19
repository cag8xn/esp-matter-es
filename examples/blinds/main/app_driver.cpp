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
uint16_t busy_endpoint = 777;
bool requested = false;

// --- Configuration ---

#define TOP_LIMIT_SWITCH_PIN GPIO_NUM_16
#define BOTTOM_LIMIT_SWITCH_PIN GPIO_NUM_17

#define STEPS_PER_REVOLUTION 200 // Adjust based on your motor
#define MICROSTEPPING 1        // Adjust based on your DRV8825 settings
#define DEFAULT_STEP_DELAY_US 800 // Adjust for desired speed

#define direction_up 1
#define direction_down 0


class StepperConfig {
public:
    gpio_num_t top_enable_pin;
    gpio_num_t top_dir_pin;
    gpio_num_t top_step_pin;
    volatile int top_current_position_steps = 0;
    uint16_t top_endpoint;
    gpio_num_t bot_enable_pin;
    gpio_num_t bot_dir_pin;
    gpio_num_t bot_step_pin;
    int max_steps = 0;  // ELIA: assuming the travel distance is the same for the top and bottom blinds
    volatile int bot_current_position_steps = 0;
    uint16_t bot_endpoint;

    bool is_calibrated = false;
};

StepperConfig Config;

struct MovementParams {
    bool bottom;
    u_int16_t position_percent;
};


class BlindDriver {
private:
    void step_motor(bool bottom, bool direction, int steps, int delay_us);
    bool is_top_limit_reached();
    bool is_bottom_limit_reached();
    int percent_to_steps(bool bottom, uint16_t percent);
    uint16_t steps_to_percent(int steps);

public:
    BlindDriver();
    void calibrate();
    void move_to_percent(bool bottom, uint16_t target_percent);
    uint16_t get_current_percent(bool bottom);
    void init();
};

BlindDriver::BlindDriver() {}


// --- Limit Switch Readings ---
bool BlindDriver::is_top_limit_reached() {
    return gpio_get_level(TOP_LIMIT_SWITCH_PIN) == 0; // Assuming switch pulls low when active
}

bool BlindDriver::is_bottom_limit_reached() {
    return gpio_get_level(BOTTOM_LIMIT_SWITCH_PIN) == 0; // Assuming switch pulls low when active
}

// --- Stepper Motor Control ---
void BlindDriver::step_motor(bool bottom, bool direction, int steps, int delay_us) {
    if (!(Config.is_calibrated)) {
        ESP_LOGW(TAG, "System not calibrated, cannot move.");
        return;
    }
    int steps_moved = 0;
    gpio_num_t step_pin = Config.top_step_pin;
    gpio_num_t enable_pin = Config.top_enable_pin;

    if (bottom) {
        step_pin = Config.bot_step_pin;
        enable_pin = Config.bot_enable_pin;
        gpio_set_level(Config.bot_dir_pin, direction);
    }
    else {
        step_pin = Config.top_step_pin;
        enable_pin = Config.top_enable_pin;
        gpio_set_level(Config.top_dir_pin, direction);
    }

    gpio_set_level(enable_pin, 0); // Enable the motor
    while (steps_moved < steps) {
        // Safety check for limit switches
        if ((direction && is_bottom_limit_reached()) || (!(direction) && is_top_limit_reached())) {
            ESP_LOGW(TAG, "Limit switch triggered, stopping movement.");
            break;
        }
        gpio_set_level(step_pin, 1);
        usleep(delay_us);
        gpio_set_level(step_pin, 0);
        usleep(delay_us);

        steps_moved++;

        if (steps_moved % 50 == 0) {
            vTaskDelay(pdMS_TO_TICKS(5)); // Micro pause (1 tick = typically 1ms)
            if (requested) {
                ESP_LOGW(TAG, "Another command was received. Current task will be terminated.");
                break;
            }
        }
        if (direction) {
            if (bottom) {
                Config.bot_current_position_steps--;
            }
            else {
                Config.top_current_position_steps--;
                if (Config.top_current_position_steps <= Config.bot_current_position_steps) {
                    break;
                }
            }
        }
        else {
            if (bottom) {
                Config.bot_current_position_steps++;
            }
            else {
                Config.top_current_position_steps++;
            }
        }
    }
    
    gpio_set_level(enable_pin, 1); // Disable the motor
    ESP_LOGI(TAG, "Stepper %d Moved %d steps in direction %d. Current positions: top %d - bottom %d", bottom, steps_moved, direction, Config.top_current_position_steps, Config.bot_current_position_steps);
}

// --- Position Conversion ---
int BlindDriver::percent_to_steps(bool bottom, uint16_t percent) {
    if (!(Config.is_calibrated) || Config.max_steps == 0) {
        ESP_LOGW(TAG, "Cannot convert percentage to steps, motor not calibrated.");
        if (bottom) {
            return Config.bot_current_position_steps; // Return current position if not calibrated
        }
        else {
            return Config.top_current_position_steps; // Return current position if not calibrated
        }
    }
    return static_cast<int>(std::round(static_cast<double>(percent) * Config.max_steps / 10000.0));
}

uint16_t BlindDriver::steps_to_percent(int steps) {
    if (!(Config.is_calibrated) || Config.max_steps == 0) {
        ESP_LOGW(TAG, "Cannot convert steps to percentage, motor not calibrated.");
        return 0;
    }
    return static_cast<uint16_t>(std::round(static_cast<double>(steps) * 10000.0 / Config.max_steps));
}

// --- Calibration Function ---
void BlindDriver::calibrate() {
    ESP_LOGI(TAG, "Starting blind calibration...");

    // Move down until bottom limit is reached
    ESP_LOGI(TAG, "Moving bottom stepper up to find top limit...");
    gpio_set_level(Config.bot_enable_pin, 0);
    gpio_set_level(Config.bot_dir_pin, direction_up); // Set direction down
    int useless_steps = 0;
    while (!is_top_limit_reached()) {
        gpio_set_level(Config.bot_step_pin, 1);
        usleep(DEFAULT_STEP_DELAY_US);
        gpio_set_level(Config.bot_step_pin, 0);
        usleep(DEFAULT_STEP_DELAY_US);
        useless_steps++;
        if (useless_steps % 50 == 0) {
            vTaskDelay(pdMS_TO_TICKS(5)); // Micro pause (1 tick = typically 1ms)
        }
    }
    gpio_set_level(Config.bot_enable_pin, 1);
    //Config.bot_current_position_steps = 0; // Reset position to 0 at bottom limit
    ESP_LOGI(TAG, "Bottom stepper top limit reached.");
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait a bit

    ESP_LOGI(TAG, "Moving bottom stepper down to find bottom limit...");
    gpio_set_level(Config.bot_enable_pin, 0);
    gpio_set_level(Config.bot_dir_pin, direction_down); // Set direction down
    int steps_counted = 0;
    while (!is_bottom_limit_reached()) {
        gpio_set_level(Config.bot_step_pin, 1);
        usleep(DEFAULT_STEP_DELAY_US);
        gpio_set_level(Config.bot_step_pin, 0);
        usleep(DEFAULT_STEP_DELAY_US);
        steps_counted++;
        if (steps_counted % 50 == 0) {
            vTaskDelay(pdMS_TO_TICKS(5)); // Micro pause (1 tick = typically 1ms)
        }
    }
    gpio_set_level(Config.bot_enable_pin, 1);
    Config.max_steps = steps_counted;
    // Config.bot_current_position_steps = 0; // Reset position to 0 at bottom limit
    Config.bot_current_position_steps = steps_counted; // Reset position to 0 at bottom limit

    ESP_LOGI(TAG, "Bottom stepper bottom limit reached. Max steps: %d", Config.max_steps);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait a bit

    ESP_LOGI(TAG, "Moving the top stepper up to find top limit...");
    gpio_set_level(Config.top_enable_pin, 0);
    gpio_set_level(Config.top_dir_pin, direction_up); // Set direction down
    useless_steps = 0;
    while (!is_top_limit_reached()) {
        gpio_set_level(Config.top_step_pin, 1);
        usleep(DEFAULT_STEP_DELAY_US);
        gpio_set_level(Config.top_step_pin, 0);
        usleep(DEFAULT_STEP_DELAY_US);
        useless_steps++;
        if (useless_steps % 50 == 0) {
            vTaskDelay(pdMS_TO_TICKS(5)); // Micro pause (1 tick = typically 1ms)
        }
    }
    gpio_set_level(Config.top_enable_pin, 1);
    ESP_LOGI(TAG, "Top stepper top limit reached.");

    //Config.top_current_position_steps = Config.max_steps;
    Config.top_current_position_steps = 0;
    Config.is_calibrated = true;
}

// --- Move to Target Position Function ---
void BlindDriver::move_to_percent(bool bottom, uint16_t target_percent) {
    if (!(Config.is_calibrated)) {
        ESP_LOGW(TAG, "Cannot move to target, motor not calibrated.");
        return;
    }

    int target_steps = percent_to_steps(bottom, target_percent);
    ESP_LOGI(TAG, "---------- Moving stepper %d to target percentage: %d (steps: %d). Current position: top %d - bottom %d",
             bottom, target_percent, target_steps, Config.top_current_position_steps, Config.bot_current_position_steps);
    int steps_to_move = 0;
    if (bottom) {
        steps_to_move = target_steps - Config.bot_current_position_steps;
    }
    else {
        steps_to_move = target_steps - Config.top_current_position_steps;
    }
    bool direction = (steps_to_move < 0);
    int abs_steps_to_move = std::abs(steps_to_move);

    if (abs_steps_to_move > 0) {
        step_motor(bottom, direction, abs_steps_to_move, DEFAULT_STEP_DELAY_US);
    } else {
        ESP_LOGI(TAG, "Target position is the same as current position.");
    }
    ESP_LOGI(TAG, "---------- Moved stepper %d to target percentage: %d (steps: %d). Current position: top %d - bottom %d",
        bottom, target_percent, target_steps, Config.top_current_position_steps, Config.bot_current_position_steps);
}

// --- Get Current Position ---
uint16_t BlindDriver::get_current_percent(bool bottom) {
    if (bottom) {
        return steps_to_percent(Config.bot_current_position_steps);
    }
    else {
        return steps_to_percent(Config.top_current_position_steps);
    }
}

// --- Global Instance of the Driver ---
static BlindDriver blind_driver;

uint16_t blind_driver_get_current_percent(bool bottom) {
    return blind_driver.get_current_percent(bottom);
}

void update_position_attribute(bool bottom) {
    //using namespace chip::app::Clusters::WindowCovering;
    using namespace chip::app::Clusters::WindowCovering;

    esp_matter_attr_val_t attr_val;
    attr_val.type = ESP_MATTER_VAL_TYPE_UINT16;  // Set the value type
    attr_val.val.u16 = blind_driver_get_current_percent(bottom);             // Store the new position
    uint16_t endpoint = 0;
    if (bottom) {
        endpoint = Config.bot_endpoint;
    }
    else {
        endpoint = Config.top_endpoint;
    }
    esp_matter::attribute::update(
        endpoint,
        WindowCovering::Id,  // Pass the cluster ID (Window Covering cluster)
        Attributes::CurrentPositionLiftPercent100ths::Id,  // Attribute ID
        &attr_val  // Pass the struct instead of a raw pointer
    );
}

void blind_driver_calibrate() {
    blind_driver.calibrate();
    update_position_attribute(true);
    update_position_attribute(false);
}

void blind_driver_move_to_percent(bool bottom, uint16_t target_percent) {
    blind_driver.move_to_percent(bottom, target_percent);
    update_position_attribute(bottom);
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

/*
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

*/
static void app_driver_button_toggle_cb(void *arg, void *data)
{
    ESP_LOGI(TAG, "Toggle button pressed");
    toggle_OnOff_attribute();
}


void calibration_task(void* param) {
    //CalibrationParams* params = (CalibrationParams*) param;
    // Call the calibration function with both steppers
    blind_driver_calibrate();
    //free(params);  // Clean up memory
    vTaskDelete(NULL); // End the task
}


void movement_task(void* param) {
    MovementParams* p = static_cast<MovementParams*>(param);
    while (busy_endpoint != 777) {
        if ((p->bottom && (busy_endpoint == blinds_endpoint_id)) || (!(p->bottom) && (busy_endpoint == blinds_top_endpoint_id))) {
            if (!(requested)) {
                requested = true;
            }
        }
        ESP_LOGI(TAG, "A driver is busy. Waiting 200ms...");
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGI(TAG, "8====================================================D DRIVER GOT BUSY!!!!!!");
    requested = false;
    if (p->bottom) {
        busy_endpoint = blinds_endpoint_id;
    }
    else {
        busy_endpoint = blinds_top_endpoint_id;
    }
    MovementParams* params = (MovementParams*) param;
    blind_driver_move_to_percent(params->bottom, params->position_percent);
    busy_endpoint = 777;
    ESP_LOGI(TAG, "8====================================================D DRIVER GOT RELEASED!!!!!!");
    free(params);  // Clean up memory
    vTaskDelete(NULL); // End the task
}



esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val)
{
    ESP_LOGI(TAG, "8===============================D : endpoint_id = %d, cluster_id = %lu, attribute_id = %lu", endpoint_id, cluster_id, attribute_id);
    //ESP_LOGI(TAG, "8===============================D : endpoint_id = %d, blinds_endpoint_id = %d", endpoint_id, blinds_endpoint_id);
    esp_err_t err = ESP_OK;

    if (cluster_id == WindowCovering::Id) {
        if (attribute_id == WindowCovering::Attributes::TargetPositionLiftPercent100ths::Id) {
            bool bottom = true;
            if (endpoint_id == blinds_endpoint_id) {
                bottom = true;
            }
            else if (endpoint_id == blinds_top_endpoint_id) {
                bottom = false;
            }
            // Adjust the lift percentage if needed
            uint16_t target_percent;
            // Access the uint16_t value from the esp_matter_attr_val_t structure
            target_percent = val->val.u16;
            //test_movement_task(bottom, target_percent);
            ESP_LOGI(TAG, "8===============================D Received attribute update: TargetPositionLiftPercent100ths - target_percent: %d", target_percent);
            
            MovementParams* params = (MovementParams*) malloc(sizeof(MovementParams));
            params->bottom = bottom;
            params->position_percent = target_percent;
            xTaskCreate(&movement_task, "movement_task", 4096, params, 5, NULL);
                  
        }
    }
    return err;
}

esp_err_t app_driver_blinds_init() {

    // Initialize the members of the object
    Config.top_step_pin = GPIO_NUM_12;   // Example GPIO for STEP
    Config.top_dir_pin = GPIO_NUM_14;    // Example GPIO for DIR
    Config.top_enable_pin = GPIO_NUM_13;  // Example GPIO for ENABLE

    // You can also create another StepperConfig object for the top lift
    Config.bot_step_pin = GPIO_NUM_26;
    Config.bot_dir_pin = GPIO_NUM_25;
    Config.bot_enable_pin = GPIO_NUM_27;


    ESP_LOGI(TAG, "Bottom Blind 1 STEP Pin: %d", Config.bot_step_pin);
    ESP_LOGI(TAG, "Top Blind 1 STEP Pin: %d", Config.top_step_pin);

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 
        (1ULL << Config.bot_step_pin) | 
        (1ULL << Config.bot_dir_pin) | 
        (1ULL << Config.bot_enable_pin) | 
        (1ULL << Config.top_step_pin) |
        (1ULL << Config.top_dir_pin) |
        (1ULL << Config.top_enable_pin);
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_set_level(Config.bot_step_pin, 0);
    gpio_set_level(Config.bot_dir_pin, 0);
    gpio_set_level(Config.bot_enable_pin, 1);
    gpio_set_level(Config.top_step_pin, 0);
    gpio_set_level(Config.top_dir_pin, 0);
    gpio_set_level(Config.top_enable_pin, 1);

    // Configure limit switch pins as inputs with pull-up
    gpio_config_t io_conf2 = {};
    io_conf2.mode = GPIO_MODE_INPUT;
    io_conf2.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf2.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf2.pin_bit_mask = (1ULL << TOP_LIMIT_SWITCH_PIN) | (1ULL << BOTTOM_LIMIT_SWITCH_PIN);
    ESP_ERROR_CHECK(gpio_config(&io_conf2));

    // Configure the correct endpoint_id for each stepper
    Config.bot_endpoint = blinds_endpoint_id;
    Config.top_endpoint = blinds_top_endpoint_id;


    ESP_LOGI(TAG, "8============D Hardware configuration complete!");

    vTaskDelay(pdMS_TO_TICKS(1000));
    
    //CalibrationParams* params = (CalibrationParams*) malloc(sizeof(CalibrationParams));
    //params->bottom_stepper = &bottomConfig1;
    //params->top_stepper = &topConfig1;
    xTaskCreate(&calibration_task, "calibration_task", 4096, NULL, 5, NULL);
    
    //blind_driver_calibrate();
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