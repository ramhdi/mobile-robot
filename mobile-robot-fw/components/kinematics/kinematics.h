#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

// Constants for the robot
#define WHEEL_RADIUS 0.05f  // 5 cm radius
#define WHEEL_BASE 0.20f    // 20 cm between wheels

typedef struct {
    float x;
    float y;
    float theta;  // Orientation angle
} pose_t;

void kinematics_init(void);
void kinematics_update(float dt);
pose_t kinematics_get_pose(void);
float kinematics_get_linear_speed(void);
float kinematics_get_angular_speed(void);

#endif  // KINEMATICS_H
