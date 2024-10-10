#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

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
