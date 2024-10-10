#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "kinematics.h"
#include "motor_control.h"
#include "pid.h"
#include "telemetry.h"
#include "udp_comm.h"

static pid_controller_t pid_left;
static pid_controller_t pid_right;
static movement_command_t current_command = {0.0f, 0.0f};

const float WHEEL_BASE = 0.20f;  // 20 cm in meters

void motor_control_task(void *arg) {
    const TickType_t xFrequency = pdMS_TO_TICKS(10);  // 100 Hz control loop
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // Compute desired wheel speeds from movement command
        float v = current_command.speed_magnitude;
        float omega = current_command.direction_angle;

        // Simple differential drive kinematics
        float v_left = v - (omega * WHEEL_BASE / 2);
        float v_right = v + (omega * WHEEL_BASE / 2);

        // Get actual wheel speeds
        float actual_speed_left =
            kinematics_get_linear_speed();  // For simplicity, using linear
                                            // speed
        float actual_speed_right = kinematics_get_linear_speed();

        // Compute PID outputs
        pid_left.setpoint = v_left;
        pid_right.setpoint = v_right;

        float control_left = pid_compute(&pid_left, actual_speed_left, 0.01f);
        float control_right =
            pid_compute(&pid_right, actual_speed_right, 0.01f);

        // Set motor speeds
        motor_set_speed(MOTOR_LEFT, control_left);
        motor_set_speed(MOTOR_RIGHT, control_right);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void kinematics_task(void *arg) {
    const TickType_t xFrequency = pdMS_TO_TICKS(10);  // 100 Hz update rate
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        kinematics_update(0.01f);  // dt = 10 ms
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void telemetry_task(void *arg) {
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 10 Hz telemetry rate
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        telemetry_send();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void command_receive_task(void *arg) {
    movement_command_t command;

    while (1) {
        if (udp_receive_command(&command)) {
            current_command = command;
            ESP_LOGI("COMMAND", "Received command: speed=%f, angle=%f",
                     command.speed_magnitude, command.direction_angle);
        }
        vTaskDelay(pdMS_TO_TICKS(50));  // Check for new command every 50 ms
    }
}

void app_main(void) {
    // Initialize components
    motor_control_init();
    kinematics_init();
    telemetry_init();

    pid_init(&pid_left, 1.0f, 0.0f, 0.0f);
    pid_init(&pid_right, 1.0f, 0.0f, 0.0f);

    // Create tasks
    xTaskCreate(motor_control_task, "Motor Control Task", 2048, NULL, 5, NULL);
    xTaskCreate(kinematics_task, "Kinematics Task", 2048, NULL, 5, NULL);
    xTaskCreate(telemetry_task, "Telemetry Task", 2048, NULL, 5, NULL);
    xTaskCreate(command_receive_task, "Command Receive Task", 2048, NULL, 5,
                NULL);
}
