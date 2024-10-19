#include "motor_simulation.h"

#include <math.h>
#include <stdlib.h>

static float motor_target_speed_left = 0.0f;
static float motor_target_speed_right = 0.0f;

static float motor_actual_speed_left = 0.0f;
static float motor_actual_speed_right = 0.0f;

void motor_simulation_init(void) {
    // Initialize motor speeds
    motor_target_speed_left = 0.0f;
    motor_target_speed_right = 0.0f;
    motor_actual_speed_left = 0.0f;
    motor_actual_speed_right = 0.0f;
}

void motor_simulation_update(float dt) {
    // Simulate left motor inertia
    float speed_difference_left =
        motor_target_speed_left - motor_actual_speed_left;
    motor_actual_speed_left +=
        (speed_difference_left * dt) / MOTOR_TIME_CONSTANT;

    // Add noise
    motor_actual_speed_left +=
        ((float)rand() / RAND_MAX - 0.5f) * MOTOR_NOISE_LEVEL;

    // Clamp speed to maximum limits
    if (motor_actual_speed_left > MOTOR_MAX_SPEED)
        motor_actual_speed_left = MOTOR_MAX_SPEED;
    if (motor_actual_speed_left < -MOTOR_MAX_SPEED)
        motor_actual_speed_left = -MOTOR_MAX_SPEED;

    // Simulate right motor inertia
    float speed_difference_right =
        motor_target_speed_right - motor_actual_speed_right;
    motor_actual_speed_right +=
        (speed_difference_right * dt) / MOTOR_TIME_CONSTANT;

    // Add noise
    motor_actual_speed_right +=
        ((float)rand() / RAND_MAX - 0.5f) * MOTOR_NOISE_LEVEL;

    // Clamp speed to maximum limits
    if (motor_actual_speed_right > MOTOR_MAX_SPEED)
        motor_actual_speed_right = MOTOR_MAX_SPEED;
    if (motor_actual_speed_right < -MOTOR_MAX_SPEED)
        motor_actual_speed_right = -MOTOR_MAX_SPEED;
}

float motor_simulation_get_speed(motor_side_t side) {
    if (side == MOTOR_LEFT) {
        return motor_actual_speed_left;
    } else {
        return motor_actual_speed_right;
    }
}

void motor_simulation_set_target_speed(motor_side_t side, float speed) {
    // Clamp target speed to [-1.0, 1.0]
    if (speed > MOTOR_MAX_SPEED) speed = MOTOR_MAX_SPEED;
    if (speed < -MOTOR_MAX_SPEED) speed = -MOTOR_MAX_SPEED;

    if (side == MOTOR_LEFT) {
        motor_target_speed_left = speed;
    } else {
        motor_target_speed_right = speed;
    }
}
