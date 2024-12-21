/**
 * @file main.c
 * @brief DC motor control application with PID feedback and serial control
 *
 * Features:
 * - MCPWM for motor PWM control
 * - Quadrature encoder feedback using PCNT
 * - PID control for speed regulation
 * - Serial interface for real-time speed control
 * - Serial debug output compatible with Serial-Studio
 */

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"
#include <math.h>

/* Debug and logging configuration */
static const char *TAG = "motor_control";
#define DEBUG_OUTPUT true

/* UART Configuration */
#define UART_PORT_NUM     UART_NUM_0
#define UART_BAUD_RATE    115200
#define UART_BUF_SIZE     256
#define UART_RX_TIMEOUT   (50 / portTICK_PERIOD_MS)

/* Motor PWM Configuration */
#define MOTOR_PWM_FREQ_HZ         25000     // 25kHz PWM frequency
#define MOTOR_PWM_RESOLUTION_HZ   10000000  // 10MHz timer resolution
#define MOTOR_PWM_DUTY_MAX        (MOTOR_PWM_RESOLUTION_HZ / MOTOR_PWM_FREQ_HZ)

/* GPIO Pin Assignments */
#define MOTOR_PWM_PIN_A   19  // Motor PWM pin A
#define MOTOR_PWM_PIN_B   18  // Motor PWM pin B
#define ENCODER_PIN_A     32  // Encoder input A
#define ENCODER_PIN_B     33  // Encoder input B

/* Encoder Configuration */
#define ENCODER_COUNT_MAX   1000
#define ENCODER_COUNT_MIN  -1000

/* PID Control Configuration */
#define PID_SAMPLE_PERIOD_MS   10   // PID update period in milliseconds
#define PID_KP  0.6f    // Proportional gain
#define PID_KI  0.4f    // Integral gain
#define PID_KD  0.2f    // Derivative gain

/* Global Variables */
static volatile int g_target_speed = 0;  // Target speed in encoder pulses
static volatile bool g_motor_enabled = false;

/**
 * @brief Motor control context structure
 */
typedef struct {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t encoder;
    pid_ctrl_block_handle_t pid;
    int current_speed;
} motor_context_t;

/**
 * @brief Initialize UART for serial communication
 */
static void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, 
                                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 
                                      UART_BUF_SIZE * 2, 0, NULL, 0));
}

/**
 * @brief Initialize motor with MCPWM
 */
static void motor_init(motor_context_t *ctx)
{
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = MOTOR_PWM_FREQ_HZ,
        .pwma_gpio_num = MOTOR_PWM_PIN_A,
        .pwmb_gpio_num = MOTOR_PWM_PIN_B,
    };
    
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = MOTOR_PWM_RESOLUTION_HZ,
    };
    
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &ctx->motor));
    ESP_ERROR_CHECK(bdc_motor_enable(ctx->motor));
}

/**
 * @brief Initialize quadrature encoder with PCNT
 */
static void encoder_init(motor_context_t *ctx)
{
    // Configure PCNT unit
    pcnt_unit_config_t unit_config = {
        .high_limit = ENCODER_COUNT_MAX,
        .low_limit = ENCODER_COUNT_MIN,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &ctx->encoder));

    // Configure glitch filter
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(ctx->encoder, &filter_config));

    // Configure channels
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENCODER_PIN_A,
        .level_gpio_num = ENCODER_PIN_B,
    };
    pcnt_channel_handle_t chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(ctx->encoder, &chan_a_config, &chan_a));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENCODER_PIN_B,
        .level_gpio_num = ENCODER_PIN_A,
    };
    pcnt_channel_handle_t chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(ctx->encoder, &chan_b_config, &chan_b));

    // Set up channel actions
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a, 
        PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Enable and start PCNT unit
    ESP_ERROR_CHECK(pcnt_unit_enable(ctx->encoder));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(ctx->encoder));
    ESP_ERROR_CHECK(pcnt_unit_start(ctx->encoder));
}

/**
 * @brief Initialize PID controller
 */
static void pid_init(motor_context_t *ctx)
{
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
 * @brief PID control loop callback
 */
static void pid_loop_cb(void *arg)
{
    static int last_count = 0;
    motor_context_t *ctx = (motor_context_t *)arg;
    
    // Get current encoder count
    int current_count = 0;
    pcnt_unit_get_count(ctx->encoder, &current_count);
    int speed = current_count - last_count;
    last_count = current_count;
    ctx->current_speed = speed;
    
    // Calculate error and update PID
    float error = abs(g_target_speed) - abs(speed);
    float new_duty = 0;
    pid_compute(ctx->pid, error, &new_duty);
    
    // Apply new duty cycle
    uint32_t duty = (uint32_t)fmin(fmax(new_duty, 0), MOTOR_PWM_DUTY_MAX - 1);
    bdc_motor_set_speed(ctx->motor, duty);
}

/**
 * @brief Serial input processing task
 */
static void serial_task(void *arg)
{
    motor_context_t *ctx = (motor_context_t *)arg;
    uint8_t data[UART_BUF_SIZE];
    char number_buf[32];
    int num_idx = 0;
    
    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data) - 1, UART_RX_TIMEOUT);
        if (len > 0) {
            data[len] = 0;
            
            for (int i = 0; i < len; i++) {
                if (data[i] == '\n' || data[i] == '\r') {
                    if (num_idx > 0) {
                        number_buf[num_idx] = 0;
                        int new_speed = atoi(number_buf);
                        
                        // Update speed and direction
                        g_target_speed = new_speed;
                        if (new_speed < 0) {
                            bdc_motor_reverse(ctx->motor);
                        } else {
                            bdc_motor_forward(ctx->motor);
                        }
                        
                        printf("Target speed set to: %d\n", new_speed);
                        num_idx = 0;
                    }
                } else if (isdigit(data[i]) || (data[i] == '-' && num_idx == 0)) {
                    if (num_idx < sizeof(number_buf) - 1) {
                        number_buf[num_idx++] = data[i];
                    }
                }
            }
        }
    }
}

/**
 * @brief Debug output task
 */
static void debug_task(void *arg)
{
    motor_context_t *ctx = (motor_context_t *)arg;
    
    while (1) {
        if (DEBUG_OUTPUT) {
            printf("/*%d,%d*/\n", ctx->current_speed, g_target_speed);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    // Create motor control context
    static motor_context_t motor_ctx = {0};
    
    ESP_LOGI(TAG, "Initializing motor control system");
    
    // Initialize all subsystems
    uart_init();
    motor_init(&motor_ctx);
    encoder_init(&motor_ctx);
    pid_init(&motor_ctx);
    
    // Create PID timer
    esp_timer_handle_t pid_timer = NULL;
    esp_timer_create_args_t timer_args = {
        .callback = pid_loop_cb,
        .arg = &motor_ctx,
        .name = "pid_loop"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &pid_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_timer, PID_SAMPLE_PERIOD_MS * 1000));
    
    // Create tasks
    xTaskCreate(serial_task, "serial_task", 4096, &motor_ctx, 5, NULL);
    xTaskCreate(debug_task, "debug_task", 2048, &motor_ctx, 4, NULL);
    
    ESP_LOGI(TAG, "System initialized and running");
    
    // Main task can now sleep
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}