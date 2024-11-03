#include "driver/uart.h"
#include "encoder.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_control.h"
#include "string.h"

#define UART_NUM UART_NUM_0
#define UART_BAUD_RATE 115200
#define UART_BUF_SIZE 1024
#define SPEED_MAX 1.0f
#define SPEED_MIN -1.0f

const char *TAG = "MAIN";

// Global variables for the motor control setpoints
volatile float control_left = 0.0f;
volatile float control_right = 0.0f;

void motor_control_task(void *arg) {
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 10 Hz control loop
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // Set motor speeds without feedback
        motor_set_speed(MOTOR_LEFT, control_left);
        // motor_set_speed(MOTOR_RIGHT, control_right);

        // Print encoder pulse counts to the Serial console
        int16_t pulse_left = encoder_get_count(ENCODER_LEFT);
        // int16_t pulse_right = encoder_get_count(ENCODER_RIGHT);

        ESP_LOGI(TAG, "Left Motor Pulse Count: %d", pulse_left);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void uart_task(void *arg) {
    uint8_t data[UART_BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(UART_NUM, data, UART_BUF_SIZE - 1,
                                  pdMS_TO_TICKS(1000));
        if (len > 0) {
            data[len] = '\0';  // Null-terminate the received data

            // Convert received data to float for speed setpoint
            float speed_setpoint = atof((char *)data);

            // Clamp speed within [-1.0, 1.0]
            if (speed_setpoint > SPEED_MAX) speed_setpoint = SPEED_MAX;
            if (speed_setpoint < SPEED_MIN) speed_setpoint = SPEED_MIN;

            // Set the same speed for both motors for testing
            control_left = speed_setpoint;
            // control_right = speed_setpoint;

            ESP_LOGI(TAG, "Set speed: %f", speed_setpoint);
        }
    }
}

void app_main(void) {
    // Initialize UART for receiving speed setpoint
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);

    // Initialize motor control and encoder
    motor_control_init();
    encoder_init();

    // Create tasks
    xTaskCreate(motor_control_task, "Motor Control Task", 2048, NULL, 5, NULL);
    xTaskCreate(uart_task, "UART Task", 2048, NULL, 5, NULL);
}
