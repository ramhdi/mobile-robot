#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float previous_error;
    float integral;
    float setpoint;
} pid_controller_t;

void pid_init(pid_controller_t *pid, float kp, float ki, float kd);
float pid_compute(pid_controller_t *pid, float measurement, float dt);

#endif  // PID_H
