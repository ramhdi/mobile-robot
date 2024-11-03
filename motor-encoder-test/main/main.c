#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "encoder.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "motor_control.h"
#include "sdkconfig.h"

#define UART_NUM UART_NUM_0
#define BUF_SIZE 1024
#define PATTERN_LEN 3

static const char *TAG = "MAIN";
static const int RX_BUF_SIZE = 1024;

// Create a queue handle
static QueueHandle_t speed_queue = NULL;

// Task for receiving UART data
void speed_receive_task(void *pvParameters) {
    uint8_t data[BUF_SIZE];
    char number_str[32];
    int number_str_len = 0;

    while (1) {
        // Read data from UART
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE, pdMS_TO_TICKS(100));

        if (len > 0) {
            // Process received data
            for (int i = 0; i < len; i++) {
                char c = (char)data[i];

                // Echo the character back to the terminal
                uart_write_bytes(UART_NUM, &c, 1);

                // Check for end of input
                if (c == '\n' || c == '\r') {
                    if (number_str_len > 0) {
                        // Null terminate the string
                        number_str[number_str_len] = '\0';

                        // Convert string to float
                        float received_number;
                        if (sscanf(number_str, "%f", &received_number) == 1) {
                            // Send the float to the processing task
                            xQueueSend(speed_queue, &received_number,
                                       portMAX_DELAY);
                        }

                        // Reset the buffer
                        number_str_len = 0;
                    }
                }
                // Store character if it's part of the number
                else if ((c >= '0' && c <= '9') || c == '.' || c == '-' ||
                         c == '+') {
                    if (number_str_len < sizeof(number_str) - 1) {
                        number_str[number_str_len++] = c;
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Task to control motor based on received speed
void motor_control_task(void *pvParameters) {
    float target_speed = 0.0f;

    while (1) {
        if (xQueueReceive(speed_queue, &target_speed, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Received speed from queue: %f", target_speed);
            motor_set_speed(MOTOR_LEFT, target_speed);
        }

        // vTaskDelayUntil(&xLastWakeTime, xFrequency);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void send_pulse_task(void *pvParameters) {
    while (1) {
        // Print encoder pulse counts to the Serial console
        int32_t pulse_left = encoder_get_count(ENCODER_LEFT);
        ESP_LOGI(TAG, "Left Motor Pulse Count: %ld", pulse_left);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void) {
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // Install UART driver
    ESP_ERROR_CHECK(
        uart_driver_install(UART_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));

    // Initialize motor control and encoder
    motor_control_init();
    encoder_init();

    // Create queue for float values
    speed_queue = xQueueCreate(10, sizeof(float));
    if (speed_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue!");
        return;
    }

    ESP_LOGI(TAG, "Let's test our motor!");

    // Create tasks
    xTaskCreate(speed_receive_task, "speed_receive_task", 2048, NULL, 5, NULL);
    xTaskCreate(motor_control_task, "motor_control_task", 2048, NULL, 4, NULL);
    // TODO: fix this line
    // xTaskCreate(send_pulse_task, "send_pulse_task", 2048, NULL, 3, NULL);
}