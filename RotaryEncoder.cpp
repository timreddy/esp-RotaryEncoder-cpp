#include <stdio.h>
#include <stdint.h>
#include <memory.h>

#include "RotaryEncoder.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define ESP_INTR_FLAG_DEFAULT 0

RotaryEncoder::RotaryEncoder(gpio_num_t pinA, gpio_num_t pinB, int min_value, int max_value) :
    pinA(pinA),
    pinB(pinB),
    min_value(min_value),
    max_value(max_value) {

    if(this->max_value <= this->min_value) {
        ESP_LOGE(tag.c_str(), "Invalid RotaryEncoder range");
    }

    this->pinA = pinA;
    this->pinB = pinB;

    this->init();
}

void RotaryEncoder::init(void) {
    this->tag = "RotaryEncoder";

    pcnt_unit_config_t pcnt_config;
    // We will use the full allowable range, and handle internally controlling the range
    pcnt_config.low_limit = INT16_MIN;
    pcnt_config.high_limit = INT16_MAX;
    pcnt_config.flags.accum_count = 0;
    pcnt_config.intr_priority = 0;

    ESP_ERROR_CHECK(pcnt_new_unit(&pcnt_config, &(this->pulse_counter)));

    pcnt_glitch_filter_config_t glitch_config;
    glitch_config.max_glitch_ns = 1000;
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(this->pulse_counter, &glitch_config));

    pcnt_chan_config_t pcnt_chan_a_config;
    pcnt_chan_a_config.edge_gpio_num              = this->pinA;
    pcnt_chan_a_config.level_gpio_num             = this->pinB;
    pcnt_chan_a_config.flags.invert_edge_input    = 0;
    pcnt_chan_a_config.flags.invert_level_input   = 0;
    pcnt_chan_a_config.flags.io_loop_back         = 0;
    pcnt_chan_a_config.flags.virt_edge_io_level   = 0;
    pcnt_chan_a_config.flags.virt_level_io_level  = 0;

    ESP_ERROR_CHECK(pcnt_new_channel(this->pulse_counter, &pcnt_chan_a_config, &(this->pulse_counter_channel_a)));

    pcnt_chan_config_t pcnt_chan_b_config;
    pcnt_chan_b_config.edge_gpio_num              = this->pinB;
    pcnt_chan_b_config.level_gpio_num             = this->pinA;
    pcnt_chan_b_config.flags.invert_edge_input    = 0;
    pcnt_chan_b_config.flags.invert_level_input   = 0;
    pcnt_chan_b_config.flags.io_loop_back         = 0;
    pcnt_chan_b_config.flags.virt_edge_io_level   = 0;
    pcnt_chan_b_config.flags.virt_level_io_level  = 0;

    ESP_ERROR_CHECK(pcnt_new_channel(this->pulse_counter, &pcnt_chan_b_config, &(this->pulse_counter_channel_b)));

    pcnt_channel_set_edge_action(this->pulse_counter_channel_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(this->pulse_counter_channel_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(this->pulse_counter_channel_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(this->pulse_counter_channel_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    ESP_ERROR_CHECK(pcnt_unit_enable(this->pulse_counter));

    ESP_ERROR_CHECK(pcnt_unit_clear_count(this->pulse_counter));
    this->current_count = 0;

    ESP_ERROR_CHECK(pcnt_unit_start(this->pulse_counter));
}

int RotaryEncoder::getCount(void) {
    int new_count;
    ESP_ERROR_CHECK(pcnt_unit_get_count(this->pulse_counter, &new_count));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(this->pulse_counter));

    this->current_count += new_count;

    // If we get to the limit, stay there.
    if(this->current_count > this->max_value) { this->current_count = this->max_value; }
    else if(this->current_count < this->min_value) { this->current_count = this->min_value; }
    return this->current_count;
}