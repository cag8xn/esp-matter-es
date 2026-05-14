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
#include "freertos/semphr.h"
#include <app_reset.h>

#include <esp_matter.h>
#include "bsp/esp-bsp.h"
#include <app_priv.h>

#include <stdio.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <unistd.h>
#include <stdbool.h>
#include <cmath> // For std::round
#include <nvs.h>
#include <nvs_flash.h>


using namespace chip::app::Clusters;
using namespace esp_matter;
using namespace esp_matter::cluster;

static const char *TAG = "app_driver";
extern uint16_t blinds_endpoint_id;
extern uint16_t blinds_top_endpoint_id;
static constexpr uint16_t INVALID_ENDPOINT_ID = UINT16_MAX;
static uint16_t busy_endpoint = INVALID_ENDPOINT_ID;
static volatile bool requested = false;
static volatile bool internal_attribute_update = false;
static uint32_t bottom_command_sequence = 0;
static uint32_t top_command_sequence = 0;
static SemaphoreHandle_t movement_mutex = NULL;
static portMUX_TYPE movement_state_lock = portMUX_INITIALIZER_UNLOCKED;

// --- Configuration ---

#define TOP_LIMIT_SWITCH_PIN GPIO_NUM_16
#define BOTTOM_LIMIT_SWITCH_PIN GPIO_NUM_17

#define STEPS_PER_REVOLUTION 200 // Adjust based on your motor
#define MICROSTEPPING 1        // Adjust based on your DRV8825 settings
#define DEFAULT_STEP_DELAY_US 400 // Adjust for desired speed
#define START_STOP_STEP_DELAY_US 1400
#define ACCELERATION_RAMP_STEPS 1000
#define DEFAULT_MAX_TRAVEL_STEPS 100000 // Increased to allow longer travel
#define LIMIT_SWITCH_DEBOUNCE_SAMPLES 20
#define LIMIT_SWITCH_DEBOUNCE_DELAY_US 1500
#define LIMIT_SWITCH_POSITION_MARGIN_MIN_STEPS 50
#define MOVEMENT_COMMAND_SETTLE_MS 1200

#define direction_up 1
#define direction_down 0


class StepperConfig {
public:
    gpio_num_t top_enable_pin;
    gpio_num_t top_dir_pin;
    gpio_num_t top_step_pin;
    int32_t top_current_position_steps = 0;
    uint16_t top_endpoint;
    gpio_num_t bot_enable_pin;
    gpio_num_t bot_dir_pin;
    gpio_num_t bot_step_pin;
    int32_t max_steps = 0;  // ELIA: assuming the travel distance is the same for the top and bottom blinds
    int32_t bot_current_position_steps = 0;
    uint16_t bot_endpoint;

    bool is_calibrated = false;
};

StepperConfig Config;

nvs_handle_t my_nvs_handle;
static bool nvs_ready = false;

struct MovementParams {
    bool bottom;
    uint16_t position_percent;
    uint32_t command_sequence;
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

static void set_requested(bool value)
{
    taskENTER_CRITICAL(&movement_state_lock);
    requested = value;
    taskEXIT_CRITICAL(&movement_state_lock);
}

static bool is_requested()
{
    bool value;
    taskENTER_CRITICAL(&movement_state_lock);
    value = requested;
    taskEXIT_CRITICAL(&movement_state_lock);
    return value;
}

static void set_busy_endpoint(uint16_t endpoint_id)
{
    taskENTER_CRITICAL(&movement_state_lock);
    busy_endpoint = endpoint_id;
    taskEXIT_CRITICAL(&movement_state_lock);
}

static uint16_t get_busy_endpoint()
{
    uint16_t endpoint_id;
    taskENTER_CRITICAL(&movement_state_lock);
    endpoint_id = busy_endpoint;
    taskEXIT_CRITICAL(&movement_state_lock);
    return endpoint_id;
}

static uint32_t register_movement_command(bool bottom)
{
    uint32_t sequence;
    taskENTER_CRITICAL(&movement_state_lock);
    if (bottom) {
        sequence = ++bottom_command_sequence;
    }
    else {
        sequence = ++top_command_sequence;
    }
    taskEXIT_CRITICAL(&movement_state_lock);
    return sequence;
}

static bool is_latest_movement_command(bool bottom, uint32_t sequence)
{
    bool is_latest;
    taskENTER_CRITICAL(&movement_state_lock);
    is_latest = sequence == (bottom ? bottom_command_sequence : top_command_sequence);
    taskEXIT_CRITICAL(&movement_state_lock);
    return is_latest;
}

static uint16_t clamp_target_percent(uint16_t percent)
{
    return percent > 10000 ? 10000 : percent;
}

static int32_t limit_switch_position_margin_steps()
{
    if (Config.max_steps <= 0) {
        return LIMIT_SWITCH_POSITION_MARGIN_MIN_STEPS;
    }

    int32_t one_percent = Config.max_steps / 100;
    return one_percent > LIMIT_SWITCH_POSITION_MARGIN_MIN_STEPS ? one_percent : LIMIT_SWITCH_POSITION_MARGIN_MIN_STEPS;
}

static bool read_limit_switch_stable(gpio_num_t pin)
{
    if (gpio_get_level(pin) != 0) {
        return false;
    }

    for (int sample = 1; sample < LIMIT_SWITCH_DEBOUNCE_SAMPLES; sample++) {
        usleep(LIMIT_SWITCH_DEBOUNCE_DELAY_US);
        if (gpio_get_level(pin) != 0) {
            return false;
        }
    }

    return true;
}

static bool is_limit_position_plausible(bool bottom, bool direction)
{
    int32_t current_position = bottom ? Config.bot_current_position_steps : Config.top_current_position_steps;
    int32_t margin_steps = limit_switch_position_margin_steps();

    if (direction == direction_up) {
        return current_position <= margin_steps;
    }

    return current_position >= (Config.max_steps - margin_steps);
}

static int get_ramped_step_delay_us(int steps_moved, int total_steps, int cruise_delay_us)
{
    if (total_steps <= 0) {
        return cruise_delay_us;
    }

    int ramp_steps = total_steps / 2;
    if (ramp_steps > ACCELERATION_RAMP_STEPS) {
        ramp_steps = ACCELERATION_RAMP_STEPS;
    }
    if (ramp_steps <= 0) {
        return cruise_delay_us;
    }

    int steps_remaining = total_steps - steps_moved;
    int ramp_position = ramp_steps;
    if (steps_moved < ramp_steps) {
        ramp_position = steps_moved;
    }
    else if (steps_remaining < ramp_steps) {
        ramp_position = steps_remaining;
    }

    int delay_range = START_STOP_STEP_DELAY_US - cruise_delay_us;
    if (delay_range <= 0) {
        return cruise_delay_us;
    }

    return START_STOP_STEP_DELAY_US - ((delay_range * ramp_position) / ramp_steps);
}


// --- Limit Switch Readings ---
bool BlindDriver::is_top_limit_reached() {
    return read_limit_switch_stable(TOP_LIMIT_SWITCH_PIN); // Assuming switch pulls low when active
}

bool BlindDriver::is_bottom_limit_reached() {
    return read_limit_switch_stable(BOTTOM_LIMIT_SWITCH_PIN); // Assuming switch pulls low when active
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
    gpio_num_t dir_pin = Config.top_dir_pin;

    if (bottom) {
        step_pin = Config.bot_step_pin;
        enable_pin = Config.bot_enable_pin;
        dir_pin = Config.bot_dir_pin;
    }
    else {
        step_pin = Config.top_step_pin;
        enable_pin = Config.top_enable_pin;
        dir_pin = Config.top_dir_pin;
    }
    gpio_set_level(dir_pin, direction);
    gpio_set_level(enable_pin, 0); // Enable the motor

    while (steps_moved < steps) {
        // Safety check for limit switches
        if (direction) {
            bool limit_position_plausible = is_limit_position_plausible(bottom, direction);
            if (limit_position_plausible && is_top_limit_reached()) {
                ESP_LOGW(TAG, "Top limit switch triggered, reverting movement and stopping stepper.");
                gpio_set_level(dir_pin, !direction);
                while (is_top_limit_reached()) {
                    gpio_set_level(step_pin, 1);
                    usleep(delay_us);
                    gpio_set_level(step_pin, 0);
                    usleep(delay_us);

                    if (bottom) {
                        Config.bot_current_position_steps++;
                    }
                    else {
                        Config.top_current_position_steps++;
                    }
                }
                break;
            }
            else if (!limit_position_plausible && (steps_moved % 200 == 0) && gpio_get_level(TOP_LIMIT_SWITCH_PIN) == 0) {
                ESP_LOGW(TAG, "Ignored top limit signal away from expected travel end. Stepper %d position: %ld, max: %ld",
                         bottom, bottom ? Config.bot_current_position_steps : Config.top_current_position_steps,
                         Config.max_steps);
            }
        }
        if (!(direction)) {
            bool limit_position_plausible = is_limit_position_plausible(bottom, direction);
            if (limit_position_plausible && is_bottom_limit_reached()) {
                ESP_LOGW(TAG, "Bottom limit switch triggered, reverting movement and stopping stepper.");
                gpio_set_level(dir_pin, !direction);
                while (is_bottom_limit_reached()) {
                    gpio_set_level(step_pin, 1);
                    usleep(delay_us);
                    gpio_set_level(step_pin, 0);
                    usleep(delay_us);

                    if (bottom) {
                        Config.bot_current_position_steps--;
                    }
                    else {
                        Config.top_current_position_steps--;
                    }
                }
                break;
            }
            else if (!limit_position_plausible && (steps_moved % 200 == 0) && gpio_get_level(BOTTOM_LIMIT_SWITCH_PIN) == 0) {
                ESP_LOGW(TAG, "Ignored bottom limit signal away from expected travel end. Stepper %d position: %ld, max: %ld",
                         bottom, bottom ? Config.bot_current_position_steps : Config.top_current_position_steps,
                         Config.max_steps);
            }
        }

        int current_delay_us = get_ramped_step_delay_us(steps_moved, steps, delay_us);
        gpio_set_level(step_pin, 1);
        usleep(current_delay_us);
        gpio_set_level(step_pin, 0);
        usleep(current_delay_us);
        steps_moved++;

        if (steps_moved % 50 == 0) {
            vTaskDelay(pdMS_TO_TICKS(5)); // Micro pause (1 tick = typically 1ms)
            if (is_requested()) {
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
    ESP_LOGI(TAG, "---------- step_motor done. Stepper %d Moved %d steps in direction %d. Current positions: top %ld - bottom %ld", bottom, steps_moved, direction, Config.top_current_position_steps, Config.bot_current_position_steps);
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
    Config.is_calibrated = false;

    // Move down until bottom limit is reached
    ESP_LOGI(TAG, "Moving bottom stepper up to find top limit...");
    gpio_set_level(Config.bot_enable_pin, 0);
    gpio_set_level(Config.bot_dir_pin, direction_up); // Set direction up
    int useless_steps = 0;
    while (!is_top_limit_reached()) {
        if (useless_steps >= DEFAULT_MAX_TRAVEL_STEPS) {
            ESP_LOGE(TAG, "Calibration failed: bottom stepper did not reach top limit within %d steps.", DEFAULT_MAX_TRAVEL_STEPS);
            gpio_set_level(Config.bot_enable_pin, 1);
            return;
        }
        gpio_set_level(Config.bot_step_pin, 1);
        usleep(DEFAULT_STEP_DELAY_US);
        gpio_set_level(Config.bot_step_pin, 0);
        usleep(DEFAULT_STEP_DELAY_US);
        useless_steps++;
        if (useless_steps % 50 == 0) {
            vTaskDelay(pdMS_TO_TICKS(5)); // Micro pause (1 tick = typically 1ms)
        }
    }
    // Move away from the limit switch
    gpio_set_level(Config.bot_dir_pin, direction_down);
    useless_steps = 0;
    while (is_top_limit_reached()) {
        if (useless_steps >= DEFAULT_MAX_TRAVEL_STEPS) {
            ESP_LOGE(TAG, "Calibration failed: bottom stepper could not move away from top limit.");
            gpio_set_level(Config.bot_enable_pin, 1);
            return;
        }
        gpio_set_level(Config.bot_step_pin, 1);
        usleep(DEFAULT_STEP_DELAY_US);
        gpio_set_level(Config.bot_step_pin, 0);
        usleep(DEFAULT_STEP_DELAY_US);
        useless_steps++;
    }
    gpio_set_level(Config.bot_enable_pin, 1);
    ESP_LOGI(TAG, "Bottom stepper top limit reached.");
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait a bit

    ESP_LOGI(TAG, "Moving bottom stepper down to find bottom limit...");
    gpio_set_level(Config.bot_enable_pin, 0);
    gpio_set_level(Config.bot_dir_pin, direction_down); // Set direction down
    int steps_counted = 0;
    while (!is_bottom_limit_reached()) {
        if (steps_counted >= DEFAULT_MAX_TRAVEL_STEPS) {
            ESP_LOGE(TAG, "Calibration failed: bottom stepper did not reach bottom limit within %d steps.", DEFAULT_MAX_TRAVEL_STEPS);
            gpio_set_level(Config.bot_enable_pin, 1);
            return;
        }
        gpio_set_level(Config.bot_step_pin, 1);
        usleep(DEFAULT_STEP_DELAY_US);
        gpio_set_level(Config.bot_step_pin, 0);
        usleep(DEFAULT_STEP_DELAY_US);
        steps_counted++;
        if (steps_counted % 50 == 0) {
            vTaskDelay(pdMS_TO_TICKS(5)); // Micro pause (1 tick = typically 1ms)
        }
    }
    // Move away from the limit switch
    gpio_set_level(Config.bot_dir_pin, direction_up);
    useless_steps = 0;
    while (is_bottom_limit_reached()) {
        if (useless_steps >= DEFAULT_MAX_TRAVEL_STEPS) {
            ESP_LOGE(TAG, "Calibration failed: bottom stepper could not move away from bottom limit.");
            gpio_set_level(Config.bot_enable_pin, 1);
            return;
        }
        gpio_set_level(Config.bot_step_pin, 1);
        usleep(DEFAULT_STEP_DELAY_US);
        gpio_set_level(Config.bot_step_pin, 0);
        usleep(DEFAULT_STEP_DELAY_US);
        steps_counted--;
        useless_steps++;
    }
    gpio_set_level(Config.bot_enable_pin, 1);
    Config.max_steps = steps_counted;
    Config.bot_current_position_steps = steps_counted;

    ESP_LOGI(TAG, "Bottom stepper bottom limit reached. Max steps: %ld", Config.max_steps);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait a bit

    ESP_LOGI(TAG, "Moving the top stepper up to find top limit...");
    gpio_set_level(Config.top_enable_pin, 0);
    gpio_set_level(Config.top_dir_pin, direction_up); // Set direction down
    useless_steps = 0;
    while (!is_top_limit_reached()) {
        if (useless_steps >= DEFAULT_MAX_TRAVEL_STEPS) {
            ESP_LOGE(TAG, "Calibration failed: top stepper did not reach top limit within %d steps.", DEFAULT_MAX_TRAVEL_STEPS);
            gpio_set_level(Config.top_enable_pin, 1);
            return;
        }
        gpio_set_level(Config.top_step_pin, 1);
        usleep(DEFAULT_STEP_DELAY_US);
        gpio_set_level(Config.top_step_pin, 0);
        usleep(DEFAULT_STEP_DELAY_US);
        useless_steps++;
        if (useless_steps % 50 == 0) {
            vTaskDelay(pdMS_TO_TICKS(5)); // Micro pause (1 tick = typically 1ms)
        }
    }
    // Move away from the limit switch
    gpio_set_level(Config.top_dir_pin, direction_down);
    useless_steps = 0;
    while (is_top_limit_reached()) {
        if (useless_steps >= DEFAULT_MAX_TRAVEL_STEPS) {
            ESP_LOGE(TAG, "Calibration failed: top stepper could not move away from top limit.");
            gpio_set_level(Config.top_enable_pin, 1);
            return;
        }
        gpio_set_level(Config.top_step_pin, 1);
        usleep(DEFAULT_STEP_DELAY_US);
        gpio_set_level(Config.top_step_pin, 0);
        usleep(DEFAULT_STEP_DELAY_US);
        useless_steps++;
    }
    gpio_set_level(Config.top_enable_pin, 1);
    ESP_LOGI(TAG, "Top stepper top limit reached.");


    Config.top_current_position_steps = 0;
    Config.is_calibrated = true;

    if (nvs_ready) {
        // Store the positions in nvs
        esp_err_t err = nvs_set_i32(my_nvs_handle, "bot_pos_step", (int32_t)Config.bot_current_position_steps);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error setting current position in NVS: %s", esp_err_to_name(err));
        }
        err = nvs_set_i32(my_nvs_handle, "top_pos_step", (int32_t)Config.top_current_position_steps);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error setting current position in NVS: %s", esp_err_to_name(err));
        }
        err = nvs_set_i32(my_nvs_handle, "max_steps", (int32_t)Config.max_steps);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error setting current position in NVS: %s", esp_err_to_name(err));
        }
        nvs_commit(my_nvs_handle);
    }
}

// --- Move to Target Position Function ---
void BlindDriver::move_to_percent(bool bottom, uint16_t target_percent) {
    if (!(Config.is_calibrated)) {
        ESP_LOGW(TAG, "Cannot move to target, motor not calibrated.");
        return;
    }

    target_percent = clamp_target_percent(target_percent);
    int target_steps = percent_to_steps(bottom, target_percent);
    ESP_LOGI(TAG, "---------- Moving stepper %d to target percentage: %d (steps: %d). Current position: top %ld - bottom %ld",
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
        // TOPS blind needs to stay ABOVE the level of the bottom blind
        if (bottom && target_steps < Config.top_current_position_steps) {
            ESP_LOGI(TAG, "---------- Requested position mismatch. Top stepper needs to move first");
            int top_steps_to_move = Config.top_current_position_steps - target_steps;
            step_motor(false, direction_up, top_steps_to_move, DEFAULT_STEP_DELAY_US);
        }
        else if (!(bottom) && target_steps > Config.bot_current_position_steps) {
            ESP_LOGI(TAG, "---------- Requested position mismatch. Bottom stepper needs to move first");
            int bot_steps_to_move = target_steps - Config.bot_current_position_steps;
            step_motor(true, direction_down, bot_steps_to_move, DEFAULT_STEP_DELAY_US);
        }
        step_motor(bottom, direction, abs_steps_to_move, DEFAULT_STEP_DELAY_US);
    } else {
        ESP_LOGI(TAG, "Target position is the same as current position.");
    }

    ESP_LOGI(TAG, "---------- Moved stepper %d to target percentage: %d (steps: %d). Current position: top %ld - bottom %ld",
        bottom, target_percent, target_steps, Config.top_current_position_steps, Config.bot_current_position_steps);
    if (nvs_ready) {
        // Store the position in nvs
        esp_err_t res = nvs_set_i32(my_nvs_handle, "bot_pos_step", (int32_t)Config.bot_current_position_steps);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Error setting current position in NVS: %s", esp_err_to_name(res));
        }
        res = nvs_set_i32(my_nvs_handle, "top_pos_step", (int32_t)Config.top_current_position_steps);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Error setting current position in NVS: %s", esp_err_to_name(res));
        }
        nvs_commit(my_nvs_handle);
    }
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
    esp_matter_attr_val_t operational_status;
    operational_status.type = ESP_MATTER_VAL_TYPE_UINT8;
    operational_status.val.u8 = 0;
    uint16_t endpoint = 0;
    if (bottom) {
        endpoint = Config.bot_endpoint;
    }
    else {
        endpoint = Config.top_endpoint;
    }
    internal_attribute_update = true;
    esp_matter::attribute::update(
        endpoint,
        WindowCovering::Id,  // Pass the cluster ID (Window Covering cluster)
        Attributes::CurrentPositionLiftPercent100ths::Id,  // Attribute ID
        &attr_val  // Pass the struct instead of a raw pointer
    );
    esp_matter::attribute::update(
        endpoint,
        WindowCovering::Id,  // Pass the cluster ID (Window Covering cluster)
        Attributes::TargetPositionLiftPercent100ths::Id,  // Attribute ID
        &attr_val  // Pass the struct instead of a raw pointer
    );
    esp_matter::attribute::update(
        endpoint,
        WindowCovering::Id,
        Attributes::OperationalStatus::Id,
        &operational_status
    );
    internal_attribute_update = false;
}

void blind_driver_calibrate() {
    blind_driver.calibrate();
    update_position_attribute(true);
    vTaskDelay(pdMS_TO_TICKS(300));
    update_position_attribute(false);
}

void blind_driver_move_to_percent(bool bottom, uint16_t target_percent) {
    blind_driver.move_to_percent(bottom, target_percent);
    update_position_attribute(true);
    vTaskDelay(pdMS_TO_TICKS(700));
    update_position_attribute(false);
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

void calibration_task(void* param) {
    if (movement_mutex == NULL) {
        ESP_LOGE(TAG, "Movement mutex is not initialized.");
        vTaskDelete(NULL);
        return;
    }

    while (xSemaphoreTake(movement_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        set_requested(true);
        ESP_LOGI(TAG, "A driver is busy. Waiting to calibrate...");
    }
    set_requested(false);
    set_busy_endpoint(INVALID_ENDPOINT_ID);
    blind_driver_calibrate();
    set_busy_endpoint(INVALID_ENDPOINT_ID);
    xSemaphoreGive(movement_mutex);
    vTaskDelete(NULL); // End the task
}

static void app_driver_button_toggle_cb(void *arg, void *data)
{
    ESP_LOGI(TAG, "Toggle button pressed");
    vTaskDelay(pdMS_TO_TICKS(1000));
    xTaskCreate(&calibration_task, "calibration_task", 4096, NULL, 5, NULL);
}

void movement_task(void* param) {
    MovementParams* p = static_cast<MovementParams*>(param);
    uint16_t requested_endpoint = p->bottom ? blinds_endpoint_id : blinds_top_endpoint_id;

    if (movement_mutex == NULL) {
        ESP_LOGE(TAG, "Movement mutex is not initialized.");
        free(p);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Waiting %dms for movement command to settle on endpoint %d.", MOVEMENT_COMMAND_SETTLE_MS,
             requested_endpoint);
    vTaskDelay(pdMS_TO_TICKS(MOVEMENT_COMMAND_SETTLE_MS));
    if (!is_latest_movement_command(p->bottom, p->command_sequence)) {
        ESP_LOGI(TAG, "Discarding superseded movement command for endpoint %d.", requested_endpoint);
        free(p);
        vTaskDelete(NULL);
        return;
    }

    while (xSemaphoreTake(movement_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        if (!is_latest_movement_command(p->bottom, p->command_sequence)) {
            ESP_LOGI(TAG, "Discarding superseded movement command while waiting for endpoint %d.", requested_endpoint);
            free(p);
            vTaskDelete(NULL);
            return;
        }
        if (get_busy_endpoint() == requested_endpoint && !is_requested()) {
            set_requested(true);
        }
        ESP_LOGI(TAG, "A driver is busy. Waiting 200ms...");
    }

    ESP_LOGI(TAG, "Movement driver is busy.");
    set_requested(false);
    set_busy_endpoint(requested_endpoint);
    blind_driver_move_to_percent(p->bottom, p->position_percent);
    set_busy_endpoint(INVALID_ENDPOINT_ID);
    xSemaphoreGive(movement_mutex);
    ESP_LOGI(TAG, "Movement driver released.");
    free(p);  // Clean up memory
    vTaskDelete(NULL); // End the task
}



esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val)
{
    ESP_LOGI(TAG, "8===============================D : endpoint_id = %d, cluster_id = %lu, attribute_id = %lu", endpoint_id, cluster_id, attribute_id);
    //ESP_LOGI(TAG, "8===============================D : endpoint_id = %d, blinds_endpoint_id = %d", endpoint_id, blinds_endpoint_id);
    esp_err_t err = ESP_OK;

    if (internal_attribute_update) {
        return err;
    }

    if (cluster_id == WindowCovering::Id) {
        if (attribute_id == WindowCovering::Attributes::TargetPositionLiftPercent100ths::Id) {
            if (endpoint_id != blinds_endpoint_id && endpoint_id != blinds_top_endpoint_id) {
                return err;
            }
            
            uint32_t current_position_attribute_id = WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id;
            node_t *node = node::get();
            endpoint_t *endpoint = endpoint::get(node, endpoint_id);
            cluster_t *cluster = cluster::get(endpoint, cluster_id);
            attribute_t *attribute = attribute::get(cluster, current_position_attribute_id);
            esp_matter_attr_val_t current_val = esp_matter_invalid(NULL);
            attribute::get_val(attribute, &current_val);

            ESP_LOGW(TAG, "8===============================D : current_val.val.u16 = %u, val->val.u16 = %u", current_val.val.u16, val->val.u16);

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
            target_percent = clamp_target_percent(val->val.u16);
            //test_movement_task(bottom, target_percent);
            ESP_LOGI(TAG, "8===============================D Received attribute update: TargetPositionLiftPercent100ths - target_percent: %d", target_percent);
            
            MovementParams* params = (MovementParams*) malloc(sizeof(MovementParams));
            if (params == NULL) {
                ESP_LOGE(TAG, "Failed to allocate movement params.");
                return ESP_ERR_NO_MEM;
            }
            params->bottom = bottom;
            params->position_percent = target_percent;
            params->command_sequence = register_movement_command(bottom);
            if (get_busy_endpoint() != INVALID_ENDPOINT_ID) {
                set_requested(true);
            }
            if (xTaskCreate(&movement_task, "movement_task", 4096, params, 5, NULL) != pdPASS) {
                ESP_LOGE(TAG, "Failed to create movement task.");
                free(params);
                return ESP_ERR_NO_MEM;
            }
                  
        }
    }
    return err;
}

esp_err_t app_driver_blinds_init() {
    if (movement_mutex == NULL) {
        movement_mutex = xSemaphoreCreateMutex();
        if (movement_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create movement mutex.");
            return ESP_ERR_NO_MEM;
        }
    }

    // Initialize the ESP NVS layer.
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK && err != ESP_ERR_NVS_NO_FREE_PAGES && err != ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "nvs_flash_init() failed: %s", esp_err_to_name(err));
    };
    
    err = nvs_open("my_app", NVS_READWRITE, &my_nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        nvs_ready = true;
        ESP_LOGI(TAG, "NVS handle opened\n");
    }

    if (nvs_ready) {
        int32_t bot_position = 0;
        int32_t top_position = 0;
        int32_t max_steps = 0;
        esp_err_t bot_err = nvs_get_i32(my_nvs_handle, "bot_pos_step", &bot_position);
        esp_err_t top_err = nvs_get_i32(my_nvs_handle, "top_pos_step", &top_position);
        esp_err_t max_err = nvs_get_i32(my_nvs_handle, "max_steps", &max_steps);

        if (bot_err == ESP_OK && top_err == ESP_OK && max_err == ESP_OK && max_steps > 0) {
            Config.bot_current_position_steps = bot_position;
            Config.top_current_position_steps = top_position;
            Config.max_steps = max_steps;
            Config.is_calibrated = true;
        } else {
            Config.bot_current_position_steps = 0;
            Config.top_current_position_steps = 0;
            Config.max_steps = 0;
            Config.is_calibrated = false;
            ESP_LOGW(TAG, "No valid calibration found in NVS. Run calibration before moving blinds.");
        }
    }

    // Initialize the members of the object
    Config.top_step_pin = GPIO_NUM_12;   // Example GPIO for STEP
    Config.top_dir_pin = GPIO_NUM_14;    // Example GPIO for DIR
    Config.top_enable_pin = GPIO_NUM_13;  

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

    update_position_attribute(true);
    vTaskDelay(pdMS_TO_TICKS(300));
    update_position_attribute(false);
    
    return ESP_OK;
}


app_driver_handle_t app_driver_button_init()
{
    /* Initialize button */
    button_config_t config = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = 15,
            .active_level = 0,
        }
    };
    button_handle_t handle = iot_button_create(&config);
    iot_button_register_cb(handle, BUTTON_PRESS_DOWN, app_driver_button_toggle_cb, NULL);
    gpio_set_pull_mode((gpio_num_t)15, GPIO_PULLUP_ONLY);
    
    return (app_driver_handle_t)handle;
}

app_driver_handle_t app_driver_reset_button_init()
{
    /* Initialize button */
    button_config_t config = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = 4,
            .active_level = 0,
        }
    };
    button_handle_t handle = iot_button_create(&config);
    app_reset_button_register(handle);

    // Enable internal pull-up resistor for GPIO 17
    gpio_set_pull_mode((gpio_num_t)4, GPIO_PULLUP_ONLY);
    
    return (app_driver_handle_t)handle;
}
