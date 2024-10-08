#pragma once 

#include <string>

#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

using namespace std;

class RotaryEncoder {
    private:
        gpio_num_t pinA;
        gpio_num_t pinB;

        pcnt_unit_handle_t pulse_counter;
        pcnt_channel_handle_t pulse_counter_channel_a;
        pcnt_channel_handle_t pulse_counter_channel_b;

        string tag;

        int current_count;
        int min_value;
        int max_value;

        void init(void);

    public: 
        RotaryEncoder(gpio_num_t pinA, gpio_num_t pinB, int min_value = -1000, int max_value = 1000);

        int getCount(void);
};