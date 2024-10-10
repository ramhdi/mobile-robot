#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

typedef enum { MOTOR_LEFT, MOTOR_RIGHT } motor_side_t;

void motor_control_init(void);
void motor_set_speed(motor_side_t side, float speed);

#endif  // MOTOR_CONTROL_H
