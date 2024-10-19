#include "kinematics.h"

#include <math.h>

#include "motor_simulation.h"

static pose_t current_pose = {0.0f, 0.0f, 0.0f};
static float linear_speed = 0.0f;
static float angular_speed = 0.0f;

void kinematics_init(void) { motor_simulation_init(); }

void kinematics_update(float dt) {
    // Update motor simulation
    motor_simulation_update(dt);

    // Get the actual wheel speeds from the simulation
    float left_speed = motor_simulation_get_speed(MOTOR_LEFT);
    float right_speed = motor_simulation_get_speed(MOTOR_RIGHT);

    // Convert normalized speeds to linear velocities
    float left_velocity =
        left_speed * MOTOR_MAX_SPEED * (2 * M_PI * WHEEL_RADIUS);
    float right_velocity =
        right_speed * MOTOR_MAX_SPEED * (2 * M_PI * WHEEL_RADIUS);

    // Compute the robot's linear and angular velocities
    float v = (right_velocity + left_velocity) / 2.0f;
    float omega = (right_velocity - left_velocity) / WHEEL_BASE;

    // Update robot pose
    current_pose.theta += omega * dt;
    current_pose.x += v * cosf(current_pose.theta) * dt;
    current_pose.y += v * sinf(current_pose.theta) * dt;

    // Update speed values
    linear_speed = v;
    angular_speed = omega;
}

pose_t kinematics_get_pose(void) { return current_pose; }

float kinematics_get_linear_speed(void) { return linear_speed; }

float kinematics_get_angular_speed(void) { return angular_speed; }
