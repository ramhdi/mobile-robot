#include "encoder.h"

#include "driver/gpio.h"
#include "esp_timer.h"

#define ENCODER_LEFT_PIN GPIO_NUM_34
#define ENCODER_RIGHT_PIN GPIO_NUM_35

static volatile int32_t encoder_count_left = 0;
static volatile int32_t encoder_count_right = 0;

static void encoder_isr_handler(void* arg) {
    encoder_side_t side = (encoder_side_t)(uintptr_t)arg;  // Fix casting issue
    if (side == ENCODER_LEFT) {
        encoder_count_left++;
    } else {
        encoder_count_right++;
    }
}

void encoder_init(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };

    // Configure left encoder pin
    io_conf.pin_bit_mask = (1ULL << ENCODER_LEFT_PIN);
    gpio_config(&io_conf);
    gpio_isr_handler_add(ENCODER_LEFT_PIN, encoder_isr_handler,
                         (void*)ENCODER_LEFT);

    // Configure right encoder pin
    io_conf.pin_bit_mask = (1ULL << ENCODER_RIGHT_PIN);
    gpio_config(&io_conf);
    gpio_isr_handler_add(ENCODER_RIGHT_PIN, encoder_isr_handler,
                         (void*)ENCODER_RIGHT);
}

int32_t encoder_get_count(encoder_side_t side) {
    if (side == ENCODER_LEFT) {
        return encoder_count_left;
    } else {
        return encoder_count_right;
    }
}

void encoder_reset(encoder_side_t side) {
    if (side == ENCODER_LEFT) {
        encoder_count_left = 0;
    } else {
        encoder_count_right = 0;
    }
}