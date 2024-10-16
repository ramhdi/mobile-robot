#include "kinematics.h"

#include <math.h>

#include "encoder.h"

// Constants for the robot
#define WHEEL_RADIUS 0.05f  // 5 cm radius
#define WHEEL_BASE 0.20f    // 20 cm between wheels
#define ENCODER_PULSES_PER_REV 11
#define GEAR_RATIO 1.0f  // Assume direct drive

static pose_t current_pose = {0.0f, 0.0f, 0.0f};
static float linear_speed = 0.0f;
static float angular_speed = 0.0f;

void kinematics_init(void) { encoder_init(); }

void kinematics_update(float dt) {
    // Get the encoder counts for left and right
    int32_t left_count = encoder_get_count(ENCODER_LEFT);
    int32_t right_count = encoder_get_count(ENCODER_RIGHT);

    // Reset the encoder counts after reading
    encoder_reset(ENCODER_LEFT);
    encoder_reset(ENCODER_RIGHT);

    // Calculate the distance traveled by each wheel, incorporating direction
    float left_distance = (2 * M_PI * WHEEL_RADIUS * left_count) /
                          (ENCODER_PULSES_PER_REV * GEAR_RATIO);
    float right_distance = (2 * M_PI * WHEEL_RADIUS * right_count) /
                           (ENCODER_PULSES_PER_REV * GEAR_RATIO);

    // Compute the robot's linear and angular displacements
    float distance =
        (right_distance + left_distance) / 2.0f;  // Average distance
    float delta_theta =
        (right_distance - left_distance) / WHEEL_BASE;  // Change in orientation

    // Update robot pose
    current_pose.theta += delta_theta;
    current_pose.x += distance * cosf(current_pose.theta);
    current_pose.y += distance * sinf(current_pose.theta);

    // Update speed values
    linear_speed = distance / dt;
    angular_speed = delta_theta / dt;
}

pose_t kinematics_get_pose(void) { return current_pose; }

float kinematics_get_linear_speed(void) { return linear_speed; }

float kinematics_get_angular_speed(void) { return angular_speed; }
