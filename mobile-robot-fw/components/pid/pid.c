#include "pid.h"

void pid_init(pid_controller_t *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->previous_error = 0.0f;
    pid->integral = 0.0f;
    pid->setpoint = 0.0f;
}

float pid_compute(pid_controller_t *pid, float measurement, float dt) {
    float error = pid->setpoint - measurement;
    pid->integral += error * dt;
    float derivative = (error - pid->previous_error) / dt;
    pid->previous_error = error;

    return (pid->kp * error) + (pid->ki * pid->integral) +
           (pid->kd * derivative);
}
