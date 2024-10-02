#include <stdio.h>
#include <stdint.h>
#include <memory.h>

#include "RotaryEncoder.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define ESP_INTR_FLAG_DEFAULT 0

void IRAM_ATTR RotaryEncoder::gpio_isr_handler(void* arg)
{
    RotaryEncoder* encoder = (RotaryEncoder*)arg;
    uint8_t encoder_state = encoder->getEncoderState();
    BaseType_t task_woken = pdFALSE;
    xQueueSendFromISR(encoder->encoder_queue, &encoder_state, &task_woken);
    portYIELD_FROM_ISR(task_woken);
}

void RotaryEncoder::vTaskEncoderState(void* pvParam) {
  RotaryEncoder* encoder = (RotaryEncoder*) pvParam;

  uint8_t encoder_state;
  uint8_t new_encoder_state;

  new_encoder_state = encoder_state = encoder->getEncoderState();
  //should go: 
  // 1,1 --> 3
  // 0,1 --> 1
  // 0,0 --> 0
  // 1,0 --> 2
  // 3->1->0->2->3 CW 
  //    3->1 CW   1101  13
  //    1->0 CW   0100  4
  //    0->2 CW   0010  2
  //    2->3 CW   1011  11
  // 
  // 3->2->0->1->3 CCW
  //    3->2      14
  //    2->0      8
  //    0->1      1
  //    1->3      7
  
  //          0   1   2   3   4   5   6   7   8   9   10    11    12    13    14    15
  int delta[16] = {0, -1,  1,  0,  1,  0,  0,  -1, -1, 0,  0,    1,    0,    1,    -1,   0};

  while(1) {
     xQueueReceive(encoder->encoder_queue, &new_encoder_state, 10);
     encoder->count += delta[encoder_state << 2 | new_encoder_state];
     ESP_LOGV(encoder->tag.c_str(), "New state: (%d, %d)   New count: %d", new_encoder_state >> 1, new_encoder_state & 1, encoder->count);
     encoder_state = new_encoder_state;
  }
}

RotaryEncoder::RotaryEncoder() { 
    this->pinA = GPIO_NUM_26;
    this->pinB = GPIO_NUM_25;

    this->init();
}

RotaryEncoder::RotaryEncoder(gpio_num_t pinA, gpio_num_t pinB) {
    this->pinA = pinA;
    this->pinB = pinB;

    this->init();
}

uint8_t RotaryEncoder::getEncoderState(void) {
    return (gpio_get_level(this->pinA) << 1) | gpio_get_level(this->pinB);
}

void RotaryEncoder::init(void) {
    this->count = 0;
    this->tag = "RotaryEncoder(";
    this->tag += to_string(this->pinA);
    this->tag += ",";
    this->tag += to_string(this->pinB);
    this->tag += ")";

    gpio_config_t config;
    config.pin_bit_mask = ( (uint64_t) 1 << this->pinA) | ( (uint64_t) 1 << this->pinB);
    config.mode = GPIO_MODE_INPUT;
    config.pull_up_en = GPIO_PULLUP_ENABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_ANYEDGE;

    this->encoder_queue = xQueueCreate( 10, sizeof( uint8_t ) );

    gpio_reset_pin(this->pinA);
    gpio_reset_pin(this->pinB);
    gpio_config(&config);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(this->pinA, RotaryEncoder::gpio_isr_handler, (void*) this);
    gpio_isr_handler_add(this->pinB, RotaryEncoder::gpio_isr_handler, (void*) this);

    TaskHandle_t xHandle = NULL;
    xTaskCreate(RotaryEncoder::vTaskEncoderState, "Encoder", 5000, this, tskIDLE_PRIORITY, &xHandle );
    configASSERT( xHandle );
}

int RotaryEncoder::getCount(void) {
    return this->count;
}