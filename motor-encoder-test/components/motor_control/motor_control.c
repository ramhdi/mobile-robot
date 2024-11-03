#include "motor_control.h"

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_err.h"
#include "esp_log.h"

#define PWM_FREQ_HZ 25000     // 25 kHz PWM frequency for the L298N
#define MOTOR_MAX_SPEED 1.0f  // Max normalized speed
#define MCPWM_TIMER_RES \
    1000000  // Adjusted Timer resolution for prescaler constraints
#define GPIO_PWM_LEFT_OUT 18   // PWM pin for left motor
#define GPIO_PWM_RIGHT_OUT 19  // PWM pin for right motor
#define GPIO_IN1_LEFT 5        // IN1 pin for left motor direction
#define GPIO_IN2_LEFT 4        // IN2 pin for left motor direction
#define GPIO_IN1_RIGHT 22      // IN1 pin for right motor direction
#define GPIO_IN2_RIGHT 23      // IN2 pin for right motor direction

static mcpwm_timer_handle_t motor_timer = NULL;
static mcpwm_oper_handle_t motor_operator_left = NULL;
static mcpwm_oper_handle_t motor_operator_right = NULL;
static mcpwm_cmpr_handle_t comparator_left = NULL;
static mcpwm_cmpr_handle_t comparator_right = NULL;

void motor_control_init(void) {
    ESP_LOGI("MOTOR_CONTROL", "Initializing motor control...");

    // Timer configuration
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz =
            MCPWM_TIMER_RES,  // Adjusted to stay within prescale limits
        .period_ticks = MCPWM_TIMER_RES / PWM_FREQ_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };

    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &motor_timer));

    // Motor operator configurations for left and right motors
    mcpwm_operator_config_t operator_config = {.group_id = 0};
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &motor_operator_left));
    ESP_ERROR_CHECK(
        mcpwm_new_operator(&operator_config, &motor_operator_right));

    // Connect operators to timer
    ESP_ERROR_CHECK(
        mcpwm_operator_connect_timer(motor_operator_left, motor_timer));
    ESP_ERROR_CHECK(
        mcpwm_operator_connect_timer(motor_operator_right, motor_timer));

    // Comparator configuration for duty cycle control
    mcpwm_comparator_config_t comparator_config = {.flags.update_cmp_on_tez =
                                                       true};
    ESP_ERROR_CHECK(mcpwm_new_comparator(motor_operator_left,
                                         &comparator_config, &comparator_left));
    ESP_ERROR_CHECK(mcpwm_new_comparator(
        motor_operator_right, &comparator_config, &comparator_right));

    // Set up PWM GPIOs for each motor
    mcpwm_generator_config_t gen_config_left = {.gen_gpio_num =
                                                    GPIO_PWM_LEFT_OUT};
    mcpwm_gen_handle_t generator_left;
    ESP_ERROR_CHECK(mcpwm_new_generator(motor_operator_left, &gen_config_left,
                                        &generator_left));

    mcpwm_generator_config_t gen_config_right = {.gen_gpio_num =
                                                     GPIO_PWM_RIGHT_OUT};
    mcpwm_gen_handle_t generator_right;
    ESP_ERROR_CHECK(mcpwm_new_generator(motor_operator_right, &gen_config_right,
                                        &generator_right));

    // Set generator actions for timer and comparator events
    mcpwm_gen_timer_event_action_t timer_action = {
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .event = MCPWM_TIMER_EVENT_EMPTY,
        .action = MCPWM_GEN_ACTION_LOW};
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_left,
                                                              timer_action));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_right,
                                                              timer_action));

    mcpwm_gen_compare_event_action_t cmp_action_left = {
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .comparator = comparator_left,
        .action = MCPWM_GEN_ACTION_HIGH};
    mcpwm_gen_compare_event_action_t cmp_action_right = {
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .comparator = comparator_right,
        .action = MCPWM_GEN_ACTION_HIGH};
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        generator_left, cmp_action_left));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        generator_right, cmp_action_right));

    // Configure GPIOs for motor direction
    gpio_set_direction(GPIO_IN1_LEFT, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_IN2_LEFT, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_IN1_RIGHT, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_IN2_RIGHT, GPIO_MODE_OUTPUT);

    // Enable the timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(motor_timer));

    // Start the timer with the command to keep it running
    ESP_ERROR_CHECK(
        mcpwm_timer_start_stop(motor_timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI("MOTOR_CONTROL", "Motor control initialized.");
}

void motor_set_speed(motor_side_t side, float speed) {
    // Clamp speed to [-1.0, 1.0]
    if (speed > MOTOR_MAX_SPEED) speed = MOTOR_MAX_SPEED;
    if (speed < -MOTOR_MAX_SPEED) speed = -MOTOR_MAX_SPEED;

    uint32_t duty = (uint32_t)(fabs(speed) * (MCPWM_TIMER_RES / PWM_FREQ_HZ));

    if (side == MOTOR_LEFT) {
        mcpwm_comparator_set_compare_value(comparator_left, duty);
        gpio_set_level(GPIO_IN1_LEFT, speed >= 0 ? 1 : 0);
        gpio_set_level(GPIO_IN2_LEFT, speed >= 0 ? 0 : 1);
    } else {
        mcpwm_comparator_set_compare_value(comparator_right, duty);
        gpio_set_level(GPIO_IN1_RIGHT, speed >= 0 ? 1 : 0);
        gpio_set_level(GPIO_IN2_RIGHT, speed >= 0 ? 0 : 1);
    }
}
