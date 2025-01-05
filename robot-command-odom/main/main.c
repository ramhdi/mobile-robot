/**
 * @file main.c
 * @brief Differential drive mobile robot control with odometry
 *
 * Features:
 * - Dual MCPWM motor control for differential drive
 * - Quadrature encoder feedback using PCNT
 * - PID control for speed regulation
 * - Odometry calculations for position tracking
 * - Serial interface for motion commands
 * - Serial debug output
 */

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "bdc_motor.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "pid_ctrl.h"

/* Debug and logging configuration */
static const char *TAG = "robot_control";
#define DEBUG_OUTPUT true

/* UART Configuration */
#define UART_PORT_NUM UART_NUM_0
#define UART_BAUD_RATE 115200
#define UART_BUF_SIZE 256
#define UART_RX_TIMEOUT (50 / portTICK_PERIOD_MS)

/* Motor PWM Configuration */
#define MOTOR_PWM_FREQ_HZ 25000           // 25kHz PWM frequency
#define MOTOR_PWM_RESOLUTION_HZ 10000000  // 10MHz timer resolution
#define MOTOR_PWM_DUTY_MAX (MOTOR_PWM_RESOLUTION_HZ / MOTOR_PWM_FREQ_HZ)

/* GPIO Pin Assignments */
// Left motor
#define LEFT_MOTOR_PWM_PIN_A 19
#define LEFT_MOTOR_PWM_PIN_B 18
#define LEFT_ENCODER_PIN_A 32
#define LEFT_ENCODER_PIN_B 33

// Right motor
#define RIGHT_MOTOR_PWM_PIN_A 16
#define RIGHT_MOTOR_PWM_PIN_B 17
#define RIGHT_ENCODER_PIN_A 25
#define RIGHT_ENCODER_PIN_B 26

/* Encoder Configuration */
#define ENCODER_COUNT_MAX 32767
#define ENCODER_COUNT_MIN -32768
#define ENCODER_CPR 1200  // Counts per revolution

/* Robot Physical Parameters */
#define WHEEL_DIAMETER_M 0.065f  // Wheel diameter in meters
#define WHEEL_BASE_M 0.235f      // Distance between wheels in meters
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER_M * M_PI)
#define METERS_PER_TICK (WHEEL_CIRCUMFERENCE / ENCODER_CPR)

/* PID Control Configuration */
#define PID_SAMPLE_PERIOD_MS 10  // PID update period in milliseconds
#define PID_KP 0.6f              // Proportional gain
#define PID_KI 0.4f              // Integral gain
#define PID_KD 0.2f              // Derivative gain

/* Motion Commands */
typedef enum {
    CMD_STOP = 0,
    CMD_FORWARD,
    CMD_BACKWARD,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_RESET_ODOMETRY
} robot_command_t;

/* Robot State */
typedef struct {
    float x;          // X position in meters
    float y;          // Y position in meters
    float theta;      // Heading in radians
    float v_linear;   // Linear velocity in m/s
    float v_angular;  // Angular velocity in rad/s
} robot_state_t;

/* Motor Context */
typedef struct {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t encoder;
    pid_ctrl_block_handle_t pid;
    int current_speed;
    int target_speed;
    int32_t total_count;
    int32_t last_count;
} motor_context_t;

/* Robot Context */
typedef struct {
    motor_context_t left_motor;
    motor_context_t right_motor;
    robot_state_t state;
    QueueHandle_t cmd_queue;
    robot_command_t current_cmd;
    float target_linear_speed;   // m/s
    float target_angular_speed;  // rad/s
    bool is_enabled;
} robot_context_t;

/* Global Variables */
static robot_context_t g_robot = {0};

/**
 * @brief Initialize UART for serial communication
 */
static void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2,
                                        UART_BUF_SIZE * 2, 0, NULL, 0));
}

/**
 * @brief Initialize a single motor with MCPWM
 */
static void motor_init(motor_context_t *ctx, int pwma_pin, int pwmb_pin) {
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = MOTOR_PWM_FREQ_HZ,
        .pwma_gpio_num = pwma_pin,
        .pwmb_gpio_num = pwmb_pin,
    };

    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = MOTOR_PWM_RESOLUTION_HZ,
    };

    ESP_ERROR_CHECK(
        bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &ctx->motor));
    ESP_ERROR_CHECK(bdc_motor_enable(ctx->motor));
    ctx->target_speed = 0;
    ctx->current_speed = 0;
    ctx->total_count = 0;
    ctx->last_count = 0;
}

/**
 * @brief Initialize quadrature encoder with PCNT
 */
static void encoder_init(motor_context_t *ctx, int pin_a, int pin_b) {
    pcnt_unit_config_t unit_config = {
        .high_limit = ENCODER_COUNT_MAX,
        .low_limit = ENCODER_COUNT_MIN,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &ctx->encoder));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(ctx->encoder, &filter_config));

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = pin_a,
        .level_gpio_num = pin_b,
    };
    pcnt_channel_handle_t chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(ctx->encoder, &chan_a_config, &chan_a));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = pin_b,
        .level_gpio_num = pin_a,
    };
    pcnt_channel_handle_t chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(ctx->encoder, &chan_b_config, &chan_b));

    ESP_ERROR_CHECK(
        pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(
        pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(
        pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(
        pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(ctx->encoder));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(ctx->encoder));
    ESP_ERROR_CHECK(pcnt_unit_start(ctx->encoder));
}

/**
 * @brief Initialize PID controller for a motor
 */
static void pid_init(motor_context_t *ctx) {
    pid_ctrl_parameter_t pid_params = {
        .kp = PID_KP,
        .ki = PID_KI,
        .kd = PID_KD,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output = MOTOR_PWM_DUTY_MAX - 1,
        .min_output = 0,
        .max_integral = 1000,
        .min_integral = -1000,
    };

    pid_ctrl_config_t pid_config = {
        .init_param = pid_params,
    };

    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &ctx->pid));
}

/**
 * @brief Update motor speed from encoder counts
 */
static void update_motor_speed(motor_context_t *ctx) {
    int current_count = 0;
    pcnt_unit_get_count(ctx->encoder, &current_count);

    // Handle counter overflow/underflow
    int32_t diff = current_count - ctx->last_count;
    if (diff > ENCODER_COUNT_MAX / 2) {
        diff -= ENCODER_COUNT_MAX;
    } else if (diff < ENCODER_COUNT_MIN / 2) {
        diff += (-ENCODER_COUNT_MIN);
    }

    ctx->current_speed = diff;
    ctx->total_count += diff;
    ctx->last_count = current_count;
}

/**
 * @brief Update robot odometry
 */
static void update_odometry(robot_context_t *robot) {
    float left_distance = robot->left_motor.current_speed * METERS_PER_TICK;
    float right_distance = robot->right_motor.current_speed * METERS_PER_TICK;

    // Calculate linear and angular displacement
    float linear_displacement = (right_distance + left_distance) / 2.0f;
    float angular_displacement =
        (right_distance - left_distance) / WHEEL_BASE_M;

    // Update robot state
    robot->state.theta += angular_displacement;
    if (robot->state.theta > M_PI) {
        robot->state.theta -= 2 * M_PI;
    } else if (robot->state.theta <= -M_PI) {
        robot->state.theta += 2 * M_PI;
    }

    // Update position
    robot->state.x += linear_displacement * cosf(robot->state.theta);
    robot->state.y += linear_displacement * sinf(robot->state.theta);

    // Update velocities
    robot->state.v_linear =
        linear_displacement * (1000.0f / PID_SAMPLE_PERIOD_MS);  // m/s
    robot->state.v_angular =
        angular_displacement * (1000.0f / PID_SAMPLE_PERIOD_MS);  // rad/s
}

/**
 * @brief Set robot movement speeds
 */
static void set_robot_speeds(robot_context_t *robot, float linear_speed,
                             float angular_speed) {
    // Convert desired linear and angular speeds to wheel speeds
    float left_speed = (linear_speed - (angular_speed * WHEEL_BASE_M / 2.0f)) /
                       METERS_PER_TICK;
    float right_speed = (linear_speed + (angular_speed * WHEEL_BASE_M / 2.0f)) /
                        METERS_PER_TICK;

    // Update target speeds for PID control
    robot->left_motor.target_speed = (int)left_speed;
    robot->right_motor.target_speed = (int)right_speed;
}

/**
 * @brief Process robot movement commands
 */
static void process_command(robot_context_t *robot, robot_command_t cmd) {
    const float LINEAR_SPEED = 0.2f;   // m/s
    const float ANGULAR_SPEED = 1.0f;  // rad/s

    switch (cmd) {
        case CMD_STOP:
            set_robot_speeds(robot, 0, 0);
            break;
        case CMD_FORWARD:
            set_robot_speeds(robot, LINEAR_SPEED, 0);
            break;
        case CMD_BACKWARD:
            set_robot_speeds(robot, -LINEAR_SPEED, 0);
            break;
        case CMD_LEFT:
            set_robot_speeds(robot, 0, ANGULAR_SPEED);
            break;
        case CMD_RIGHT:
            set_robot_speeds(robot, 0, -ANGULAR_SPEED);
            break;
        case CMD_RESET_ODOMETRY:
            memset(&robot->state, 0, sizeof(robot_state_t));
            break;
    }

    robot->current_cmd = cmd;
}

/**
 * @brief PID control loop callback
 */
static void pid_loop_cb(void *arg) {
    robot_context_t *robot = (robot_context_t *)arg;

    // Update motor speeds from encoders
    update_motor_speed(&robot->left_motor);
    update_motor_speed(&robot->right_motor);

    // Update odometry
    update_odometry(robot);

    // PID control for each motor
    float left_error =
        robot->left_motor.target_speed - robot->left_motor.current_speed;
    float right_error =
        robot->right_motor.target_speed - robot->right_motor.current_speed;

    float left_duty = 0, right_duty = 0;
    pid_compute(robot->left_motor.pid, left_error, &left_duty);
    pid_compute(robot->right_motor.pid, right_error, &right_duty);

    // Apply duty cycles to motors
    uint32_t left_duty_cycle =
        (uint32_t)fmin(fmax(left_duty, 0), MOTOR_PWM_DUTY_MAX - 1);
    uint32_t right_duty_cycle =
        (uint32_t)fmin(fmax(right_duty, 0), MOTOR_PWM_DUTY_MAX - 1);

    bdc_motor_set_speed(robot->left_motor.motor, left_duty_cycle);
    bdc_motor_set_speed(robot->right_motor.motor, right_duty_cycle);
}

/**
 * @brief Serial input processing task
 */
static void serial_task(void *arg) {
    robot_context_t *robot = (robot_context_t *)arg;
    uint8_t data[UART_BUF_SIZE];

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data) - 1,
                                  UART_RX_TIMEOUT);
        if (len > 0) {
            data[len] = 0;

            // Process each character
            for (int i = 0; i < len; i++) {
                robot_command_t cmd = CMD_STOP;

                switch (tolower(data[i])) {
                    case 'w':
                        cmd = CMD_FORWARD;
                        break;
                    case 's':
                        cmd = CMD_BACKWARD;
                        break;
                    case 'a':
                        cmd = CMD_LEFT;
                        break;
                    case 'd':
                        cmd = CMD_RIGHT;
                        break;
                    case 'x':
                        cmd = CMD_STOP;
                        break;
                    case 'r':
                        cmd = CMD_RESET_ODOMETRY;
                        break;
                    default:
                        continue;
                }

                // Send command to processing queue
                xQueueSend(robot->cmd_queue, &cmd, 0);
            }
        }
    }
}

/**
 * @brief Command processing task
 */
static void command_task(void *arg) {
    robot_context_t *robot = (robot_context_t *)arg;
    robot_command_t cmd;

    while (1) {
        if (xQueueReceive(robot->cmd_queue, &cmd, portMAX_DELAY)) {
            process_command(robot, cmd);
        }
    }
}

/**
 * @brief Debug output task
 */
static void debug_task(void *arg) {
    robot_context_t *robot = (robot_context_t *)arg;

    while (1) {
        if (DEBUG_OUTPUT) {
            // Output format compatible with plotting tools
            printf("/*%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d*/\n", robot->state.x,
                   robot->state.y, robot->state.theta, robot->state.v_linear,
                   robot->state.v_angular, robot->left_motor.current_speed,
                   robot->left_motor.target_speed,
                   robot->right_motor.current_speed,
                   robot->right_motor.target_speed);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Initializing robot control system");

    // Initialize command queue
    g_robot.cmd_queue = xQueueCreate(10, sizeof(robot_command_t));

    // Initialize all subsystems
    uart_init();

    // Initialize left motor subsystem
    motor_init(&g_robot.left_motor, LEFT_MOTOR_PWM_PIN_A, LEFT_MOTOR_PWM_PIN_B);
    encoder_init(&g_robot.left_motor, LEFT_ENCODER_PIN_A, LEFT_ENCODER_PIN_B);
    pid_init(&g_robot.left_motor);

    // Initialize right motor subsystem
    motor_init(&g_robot.right_motor, RIGHT_MOTOR_PWM_PIN_A,
               RIGHT_MOTOR_PWM_PIN_B);
    encoder_init(&g_robot.right_motor, RIGHT_ENCODER_PIN_A,
                 RIGHT_ENCODER_PIN_B);
    pid_init(&g_robot.right_motor);

    // Reset robot state
    memset(&g_robot.state, 0, sizeof(robot_state_t));
    g_robot.is_enabled = true;

    // Create PID timer
    esp_timer_handle_t pid_timer = NULL;
    esp_timer_create_args_t timer_args = {
        .callback = pid_loop_cb, .arg = &g_robot, .name = "pid_loop"};
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &pid_timer));
    ESP_ERROR_CHECK(
        esp_timer_start_periodic(pid_timer, PID_SAMPLE_PERIOD_MS * 1000));

    // Create tasks
    xTaskCreate(serial_task, "serial_task", 4096, &g_robot, 5, NULL);
    xTaskCreate(command_task, "command_task", 2048, &g_robot, 5, NULL);
    xTaskCreate(debug_task, "debug_task", 2048, &g_robot, 4, NULL);

    ESP_LOGI(TAG, "Robot control system initialized and running");

    // Main task can now sleep
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}