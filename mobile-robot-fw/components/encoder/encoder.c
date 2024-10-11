#include "encoder.h"

#include "driver/gpio.h"
#include "esp_timer.h"

// Define the pins for encoder signals A and B
#define ENCODER_LEFT_A_PIN GPIO_NUM_34
#define ENCODER_LEFT_B_PIN GPIO_NUM_35
#define ENCODER_RIGHT_A_PIN GPIO_NUM_36
#define ENCODER_RIGHT_B_PIN GPIO_NUM_37

static volatile int32_t encoder_count_left = 0;
static volatile int32_t encoder_count_right = 0;

// Direction flags
static volatile bool direction_left = true;
static volatile bool direction_right = true;

static void encoder_isr_handler(void* arg) {
    encoder_side_t side = (encoder_side_t)arg;

    // Read both A and B signals
    bool A, B;

    if (side == ENCODER_LEFT) {
        A = gpio_get_level(ENCODER_LEFT_A_PIN);
        B = gpio_get_level(ENCODER_LEFT_B_PIN);
    } else {
        A = gpio_get_level(ENCODER_RIGHT_A_PIN);
        B = gpio_get_level(ENCODER_RIGHT_B_PIN);
    }

    // Determine direction based on quadrature encoding
    if (A == B) {
        // If A and B are the same, the motor is moving forward
        if (side == ENCODER_LEFT) {
            direction_left = true;
            encoder_count_left++;
        } else {
            direction_right = true;
            encoder_count_right++;
        }
    } else {
        // If A and B are different, the motor is moving backward
        if (side == ENCODER_LEFT) {
            direction_left = false;
            encoder_count_left--;
        } else {
            direction_right = false;
            encoder_count_right--;
        }
    }
}

void encoder_init(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };

    // Configure left encoder pins
    io_conf.pin_bit_mask =
        (1ULL << ENCODER_LEFT_A_PIN) | (1ULL << ENCODER_LEFT_B_PIN);
    gpio_config(&io_conf);
    gpio_isr_handler_add(ENCODER_LEFT_A_PIN, encoder_isr_handler,
                         (void*)ENCODER_LEFT);
    gpio_isr_handler_add(ENCODER_LEFT_B_PIN, encoder_isr_handler,
                         (void*)ENCODER_LEFT);

    // Configure right encoder pins
    io_conf.pin_bit_mask =
        (1ULL << ENCODER_RIGHT_A_PIN) | (1ULL << ENCODER_RIGHT_B_PIN);
    gpio_config(&io_conf);
    gpio_isr_handler_add(ENCODER_RIGHT_A_PIN, encoder_isr_handler,
                         (void*)ENCODER_RIGHT);
    gpio_isr_handler_add(ENCODER_RIGHT_B_PIN, encoder_isr_handler,
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
