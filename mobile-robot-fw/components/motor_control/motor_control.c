#include "motor_control.h"

#include "driver/mcpwm_prelude.h"
#include "esp_err.h"
#include "esp_log.h"

#define PWM_FREQ_HZ 20000     // 20 kHz PWM frequency
#define MOTOR_MAX_SPEED 1.0f  // Maximum normalized speed (between -1.0 and 1.0)
#define MCPWM_TIMER_RES 10000  // PWM resolution (duty cycle steps)
#define GPIO_PWM0A_OUT 18      // Left motor PWM output (for example, GPIO 18)
#define GPIO_PWM0B_OUT 19      // Right motor PWM output (for example, GPIO 19)

static mcpwm_timer_handle_t motor_timer = NULL;
static mcpwm_oper_handle_t motor_operator_left = NULL;
static mcpwm_oper_handle_t motor_operator_right = NULL;
static mcpwm_cmpr_handle_t comparator_left = NULL;
static mcpwm_cmpr_handle_t comparator_right = NULL;

void motor_control_init(void) {
    ESP_LOGI("MOTOR_CONTROL", "Initializing motor control...");

    // Timer configuration
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,  // Use MCPWM Group 0
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_FREQ_HZ * MCPWM_TIMER_RES,
        .period_ticks = MCPWM_TIMER_RES,          // Period based on resolution
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,  // Up-counting mode
    };

    // Initialize MCPWM timer
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &motor_timer));

    // Operator configuration for left motor
    mcpwm_operator_config_t operator_config = {
        .group_id = 0  // Use MCPWM Group 0
    };

    // Initialize left motor operator
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &motor_operator_left));

    // Initialize right motor operator
    ESP_ERROR_CHECK(
        mcpwm_new_operator(&operator_config, &motor_operator_right));

    // Connect operators to timer
    ESP_ERROR_CHECK(
        mcpwm_operator_connect_timer(motor_operator_left, motor_timer));
    ESP_ERROR_CHECK(
        mcpwm_operator_connect_timer(motor_operator_right, motor_timer));

    // Configure comparators for left motor
    mcpwm_comparator_config_t comparator_config = {
        .flags = {
            .update_cmp_on_tez = true  // Update comparator on timer zero
        }};
    ESP_ERROR_CHECK(mcpwm_new_comparator(motor_operator_left,
                                         &comparator_config, &comparator_left));

    // Configure comparators for right motor
    ESP_ERROR_CHECK(mcpwm_new_comparator(
        motor_operator_right, &comparator_config, &comparator_right));

    // Configure PWM output GPIOs
    mcpwm_generator_config_t generator_config_left = {
        .gen_gpio_num = GPIO_PWM0A_OUT,  // Left motor PWM pin
    };
    mcpwm_gen_handle_t generator_left;
    ESP_ERROR_CHECK(mcpwm_new_generator(
        motor_operator_left, &generator_config_left, &generator_left));

    mcpwm_generator_config_t generator_config_right = {
        .gen_gpio_num = GPIO_PWM0B_OUT,  // Right motor PWM pin
    };
    mcpwm_gen_handle_t generator_right;
    ESP_ERROR_CHECK(mcpwm_new_generator(
        motor_operator_right, &generator_config_right, &generator_right));

    // Configure generator action for timer events (set LOW on timer empty)
    mcpwm_gen_timer_event_action_t timer_event_action_left = {
        .direction = MCPWM_TIMER_DIRECTION_UP,  // Timer counts up
        .event = MCPWM_TIMER_EVENT_EMPTY,       // On timer zero
        .action = MCPWM_GEN_ACTION_LOW          // Set output to LOW
    };
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        generator_left, timer_event_action_left));

    mcpwm_gen_timer_event_action_t timer_event_action_right = {
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .event = MCPWM_TIMER_EVENT_EMPTY,
        .action = MCPWM_GEN_ACTION_LOW};
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        generator_right, timer_event_action_right));

    // Configure generator action for comparator events (set HIGH on compare
    // match)
    mcpwm_gen_compare_event_action_t compare_event_action_left = {
        .direction = MCPWM_TIMER_DIRECTION_UP,  // Timer counts up
        .comparator = comparator_left,          // Compare to left comparator
        .action = MCPWM_GEN_ACTION_HIGH         // Set output to HIGH
    };
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        generator_left, compare_event_action_left));

    mcpwm_gen_compare_event_action_t compare_event_action_right = {
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .comparator = comparator_right,
        .action = MCPWM_GEN_ACTION_HIGH};
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        generator_right, compare_event_action_right));

    // Start the timer with the command to keep it running
    ESP_ERROR_CHECK(
        mcpwm_timer_start_stop(motor_timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI("MOTOR_CONTROL", "Motor control initialized.");
}

void motor_set_speed(motor_side_t side, float speed) {
    // Clamp speed to [-1.0, 1.0]
    if (speed > MOTOR_MAX_SPEED) speed = MOTOR_MAX_SPEED;
    if (speed < -MOTOR_MAX_SPEED) speed = -MOTOR_MAX_SPEED;

    // Convert speed to duty cycle
    uint32_t duty = (uint32_t)((speed + 1.0f) * (MCPWM_TIMER_RES / 2));

    if (side == MOTOR_LEFT) {
        // Set duty cycle for left motor
        mcpwm_comparator_set_compare_value(comparator_left, duty);
    } else {
        // Set duty cycle for right motor
        mcpwm_comparator_set_compare_value(comparator_right, duty);
    }
}
