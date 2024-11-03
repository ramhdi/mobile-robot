#include "encoder.h"

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"

#define TAG "ENCODER"
#define ENCODER_LEFT_A_PIN GPIO_NUM_34
#define ENCODER_LEFT_B_PIN GPIO_NUM_35
#define ENCODER_RIGHT_A_PIN GPIO_NUM_36
#define ENCODER_RIGHT_B_PIN GPIO_NUM_37
#define ENCODER_PCNT_HIGH_LIMIT 1000
#define ENCODER_PCNT_LOW_LIMIT -1000
#define GLITCH_FILTER_NS 1000

static pcnt_unit_handle_t pcnt_unit_left = NULL;
static pcnt_unit_handle_t pcnt_unit_right = NULL;

void encoder_init(void) {
    ESP_LOGI(TAG, "Initializing PCNT encoder units");

    // Configure PCNT for the left encoder
    pcnt_unit_config_t left_config = {
        .high_limit = ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&left_config, &pcnt_unit_left));

    // Configure glitch filter for noise reduction
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = GLITCH_FILTER_NS,
    };
    ESP_ERROR_CHECK(
        pcnt_unit_set_glitch_filter(pcnt_unit_left, &filter_config));

    // Configure channels for left encoder
    pcnt_chan_config_t left_chan_a = {
        .edge_gpio_num = ENCODER_LEFT_A_PIN,
        .level_gpio_num = ENCODER_LEFT_B_PIN,
    };
    pcnt_channel_handle_t left_channel_a;
    ESP_ERROR_CHECK(
        pcnt_new_channel(pcnt_unit_left, &left_chan_a, &left_channel_a));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        left_channel_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        left_channel_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    pcnt_chan_config_t left_chan_b = {
        .edge_gpio_num = ENCODER_LEFT_B_PIN,
        .level_gpio_num = ENCODER_LEFT_A_PIN,
    };
    pcnt_channel_handle_t left_channel_b;
    ESP_ERROR_CHECK(
        pcnt_new_channel(pcnt_unit_left, &left_chan_b, &left_channel_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        left_channel_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        left_channel_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Enable and start the left PCNT unit
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_left));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_left));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_left));

    // Configure PCNT for the right encoder (similar to the left)
    pcnt_unit_config_t right_config = left_config;
    ESP_ERROR_CHECK(pcnt_new_unit(&right_config, &pcnt_unit_right));

    ESP_ERROR_CHECK(
        pcnt_unit_set_glitch_filter(pcnt_unit_right, &filter_config));

    pcnt_chan_config_t right_chan_a = {
        .edge_gpio_num = ENCODER_RIGHT_A_PIN,
        .level_gpio_num = ENCODER_RIGHT_B_PIN,
    };
    pcnt_channel_handle_t right_channel_a;
    ESP_ERROR_CHECK(
        pcnt_new_channel(pcnt_unit_right, &right_chan_a, &right_channel_a));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        right_channel_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        right_channel_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    pcnt_chan_config_t right_chan_b = {
        .edge_gpio_num = ENCODER_RIGHT_B_PIN,
        .level_gpio_num = ENCODER_RIGHT_A_PIN,
    };
    pcnt_channel_handle_t right_channel_b;
    ESP_ERROR_CHECK(
        pcnt_new_channel(pcnt_unit_right, &right_chan_b, &right_channel_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        right_channel_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        right_channel_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_right));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_right));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_right));

    ESP_LOGI(TAG, "PCNT encoders initialized");
}

int32_t encoder_get_count(encoder_side_t side) {
    int32_t count = 0;
    if (side == ENCODER_LEFT) {
        pcnt_unit_get_count(pcnt_unit_left, &count);
    } else if (side == ENCODER_RIGHT) {
        pcnt_unit_get_count(pcnt_unit_right, &count);
    }
    return count;
}

void encoder_reset(encoder_side_t side) {
    if (side == ENCODER_LEFT) {
        pcnt_unit_clear_count(pcnt_unit_left);
    } else if (side == ENCODER_RIGHT) {
        pcnt_unit_clear_count(pcnt_unit_right);
    }
}
