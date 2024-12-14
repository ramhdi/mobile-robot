/**
 * @file main.c
 * @brief Example application demonstrating DC motor control with PID feedback
 * using ESP32.
 *
 * This application configures MCPWM for motor control and PCNT for rotary
 * encoder feedback. A PID controller is used to maintain a constant motor
 * speed. The logging format is compatible with Serial-Studio for debugging and
 * visualization.
 *
 * @author
 * @date 2021-2022
 * @copyright
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include "bdc_motor.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "pid_ctrl.h"
#include <math.h>
#include "sdkconfig.h"

/// Log tag for this example
static const char *TAG = "example";

/// @brief If enabled, outputs formatted debug strings for Serial-Studio
#define SERIAL_STUDIO_DEBUG true

/// @brief MCPWM timer resolution (Hz)
#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us

/// @brief MCPWM frequency (Hz)
#define BDC_MCPWM_FREQ_HZ 25000 // 25KHz PWM

/// @brief Maximum MCPWM duty cycle in ticks
#define BDC_MCPWM_DUTY_TICK_MAX \
    (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ)

/// @brief MCPWM GPIO pin for PWMA
#define BDC_MCPWM_GPIO_A 19

/// @brief MCPWM GPIO pin for PWMB
#define BDC_MCPWM_GPIO_B 18

/// @brief PCNT GPIO pin for encoder channel A
#define BDC_ENCODER_GPIO_A 32

/// @brief PCNT GPIO pin for encoder channel B
#define BDC_ENCODER_GPIO_B 33

/// @brief PCNT high count limit
#define BDC_ENCODER_PCNT_HIGH_LIMIT 1000

/// @brief PCNT low count limit
#define BDC_ENCODER_PCNT_LOW_LIMIT -1000

/// @brief PID loop period in milliseconds
#define BDC_PID_LOOP_PERIOD_MS 10

/// @brief Expected motor speed in encoder pulses
#define BDC_PID_EXPECT_SPEED -25

/**
 * @brief Context for motor control, including motor, encoder, and PID handles.
 */
typedef struct
{
    bdc_motor_handle_t motor;         ///< DC motor handle
    pcnt_unit_handle_t pcnt_encoder;  ///< PCNT encoder handle
    pid_ctrl_block_handle_t pid_ctrl; ///< PID control block handle
    int report_pulses;                ///< Pulse count reported by encoder
} motor_control_context_t;

/**
 * @brief PID loop callback for periodic speed control adjustments.
 *
 * This callback calculates the motor speed error, updates the PID controller,
 * and adjusts the motor speed accordingly.
 *
 * @param args Pointer to motor_control_context_t.
 */
static void pid_loop_cb(void *args) {
    static int last_pulse_count = 0;
    static bool initial_direction_set = false;
    motor_control_context_t *ctx = (motor_control_context_t *)args;
    pcnt_unit_handle_t pcnt_unit = ctx->pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl = ctx->pid_ctrl;
    bdc_motor_handle_t motor = ctx->motor;

    // Get pulse count from rotary encoder
    int cur_pulse_count = 0;
    pcnt_unit_get_count(pcnt_unit, &cur_pulse_count);
    int real_pulses = cur_pulse_count - last_pulse_count;
    last_pulse_count = cur_pulse_count;
    ctx->report_pulses = real_pulses;

    // Set initial direction based on target (only once)
    if (!initial_direction_set) {
        if (BDC_PID_EXPECT_SPEED < 0) {
            bdc_motor_reverse(motor);
        } else {
            bdc_motor_forward(motor);
        }
        initial_direction_set = true;
        return;  // Skip first PID computation to let direction settle
    }

    // Calculate target and error using absolute values
    int target_abs = abs(BDC_PID_EXPECT_SPEED);
    int current_abs = abs(real_pulses);
    float error = target_abs - current_abs;

    // Compute new speed using PID controller
    float new_speed = 0;
    pid_compute(pid_ctrl, error, &new_speed);

    // Adjust speed and ensure it's within bounds
    uint32_t applied_speed = (uint32_t)fmin(fmax(new_speed, 0), BDC_MCPWM_DUTY_TICK_MAX - 1);
    bdc_motor_set_speed(motor, applied_speed);
}
/**
 * @brief Main application entry point.
 *
 * Initializes peripherals, sets up motor control, and starts the PID loop.
 */
void app_main(void)
{
    static motor_control_context_t motor_ctrl_ctx = {
        .pcnt_encoder = NULL,
    };

    ESP_LOGI(TAG, "Create DC motor");
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_A,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_B,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(
        bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    motor_ctrl_ctx.motor = motor;

    ESP_LOGI(TAG, "Init PCNT driver for rotary encoder");
    pcnt_unit_config_t unit_config = {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    // Configure encoder channels
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = BDC_ENCODER_GPIO_A,
        .level_gpio_num = BDC_ENCODER_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = BDC_ENCODER_GPIO_B,
        .level_gpio_num = BDC_ENCODER_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    // Set channel actions
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Add watch points and enable PCNT
    ESP_ERROR_CHECK(
        pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(
        pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
    motor_ctrl_ctx.pcnt_encoder = pcnt_unit;

    ESP_LOGI(TAG, "Create PID control block");
    pid_ctrl_parameter_t pid_runtime_param = {
        .kp = 0.6,
        .ki = 0.4,
        .kd = 0.2,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output = BDC_MCPWM_DUTY_TICK_MAX - 1,
        .min_output = 0,
        .max_integral = 1000,
        .min_integral = -1000,
    };
    pid_ctrl_block_handle_t pid_ctrl = NULL;
    pid_ctrl_config_t pid_config = {
        .init_param = pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl));
    motor_ctrl_ctx.pid_ctrl = pid_ctrl;

    ESP_LOGI(TAG, "Create timer for periodic PID updates");
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = pid_loop_cb, .arg = &motor_ctrl_ctx, .name = "pid_loop"};
    esp_timer_handle_t pid_loop_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &pid_loop_timer));

    ESP_LOGI(TAG, "Enable motor");
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    ESP_LOGI(TAG, "Set initial motor direction");
    if (BDC_PID_EXPECT_SPEED < 0)
    {
        ESP_ERROR_CHECK(bdc_motor_reverse(motor));
    }
    else
    {
        ESP_ERROR_CHECK(bdc_motor_forward(motor));
    }

    ESP_LOGI(TAG, "Start PID loop");
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer,
                                             BDC_PID_LOOP_PERIOD_MS * 1000));

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
#if SERIAL_STUDIO_DEBUG
        printf("/*%d*/\r\n", motor_ctrl_ctx.report_pulses);
#endif
    }
}
