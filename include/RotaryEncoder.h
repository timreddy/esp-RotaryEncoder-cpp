#pragma once 

#include <string>

#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"

using namespace std;

class RotaryEncoder {
    private:
        gpio_num_t pinA;
        gpio_num_t pinB;
        QueueHandle_t encoder_queue;

        int count;
        string tag;

        void init(void);

        static void vTaskEncoderState(void* pvParam);
        static void IRAM_ATTR gpio_isr_handler(void* arg);

        uint8_t getEncoderState(void);

    public: 
        RotaryEncoder();
        RotaryEncoder(gpio_num_t pinA, gpio_num_t pinB);

        int getCount(void);
};