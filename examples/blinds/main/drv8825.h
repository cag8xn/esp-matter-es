#ifndef DRV8825_H
#define DRV8825_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void drv8825_enable(gpio_num_t enable_pin, bool enable) {
    gpio_set_direction(enable_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(enable_pin, enable ? 0 : 1);  // Active low
}

void drv8825_set_direction(gpio_num_t dir_pin, bool direction) {
    gpio_set_direction(dir_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(dir_pin, direction);
}

void drv8825_step(gpio_num_t step_pin) {
    gpio_set_direction(step_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(step_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(step_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
}

#endif // DRV8825_H
