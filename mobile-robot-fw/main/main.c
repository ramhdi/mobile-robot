#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "kinematics.h"
// #include "motor_control.h"
#include <math.h>

#include "motor_simulation.h"
#include "pid.h"
#include "telemetry.h"
#include "udp_comm.h"
#include "wifi.h"

static pid_controller_t pid_left;
static pid_controller_t pid_right;
static movement_command_t current_command = {0.0f, 0.0f};

void motor_control_task(void *arg) {
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 10 Hz control loop
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        float dt = 0.1f;  // 100 ms in seconds

        // Compute desired wheel speeds from movement command
        float v = current_command.speed_magnitude;
        float omega = current_command.direction_angle;

        // Simple differential drive kinematics
        float v_left = v - (omega * WHEEL_BASE / 2);
        float v_right = v + (omega * WHEEL_BASE / 2);

        // Convert linear velocities to normalized speeds
        float target_speed_left =
            v_left / (2 * M_PI * WHEEL_RADIUS * MOTOR_MAX_SPEED);
        float target_speed_right =
            v_right / (2 * M_PI * WHEEL_RADIUS * MOTOR_MAX_SPEED);

        // Get actual wheel speeds from simulation
        float actual_speed_left = motor_simulation_get_speed(MOTOR_LEFT);
        float actual_speed_right = motor_simulation_get_speed(MOTOR_RIGHT);

        // Compute PID outputs
        pid_left.setpoint = target_speed_left;
        pid_right.setpoint = target_speed_right;

        float control_left = pid_compute(&pid_left, actual_speed_left, dt);
        float control_right = pid_compute(&pid_right, actual_speed_right, dt);

        // Set motor target speeds in simulation
        motor_simulation_set_target_speed(MOTOR_LEFT, control_left);
        motor_simulation_set_target_speed(MOTOR_RIGHT, control_right);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void kinematics_task(void *arg) {
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 10 Hz update rate
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        kinematics_update(0.01f);  // dt = 10 ms
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void telemetry_task(void *arg) {
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);  // 1 Hz telemetry rate
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        telemetry_send();

        // Log stack usage
        UBaseType_t remaining_stack = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGI("Telemetry Task", "Remaining stack: %d bytes",
                 remaining_stack * sizeof(StackType_t));

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
        vTaskDelay(pdMS_TO_TICKS(100));  // Check for new command every 100 ms
    }
}

void app_main(void) {
    // Initialize Wi-Fi and wait for connection
    wifi_init_sta();

    // Initialize components
    // motor_control_init();
    motor_simulation_init();
    kinematics_init();
    telemetry_init();

    pid_init(&pid_left, 1.0f, 0.0f, 0.0f);
    pid_init(&pid_right, 1.0f, 0.0f, 0.0f);

    // Create tasks
    xTaskCreate(motor_control_task, "Motor Control Task", 2048, NULL, 5, NULL);
    xTaskCreate(kinematics_task, "Kinematics Task", 2048, NULL, 5, NULL);
    xTaskCreate(telemetry_task, "Telemetry Task", 3072, NULL, 5, NULL);
    xTaskCreate(command_receive_task, "Command Receive Task", 2048, NULL, 5,
                NULL);
}
