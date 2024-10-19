#ifndef MOTOR_SIMULATION_H
#define MOTOR_SIMULATION_H

#define MOTOR_MAX_SPEED 1.0f  // Maximum normalized speed (between -1.0 and 1.0)
#define MOTOR_TIME_CONSTANT 0.1f  // Time constant for motor inertia
#define MOTOR_NOISE_LEVEL 0.05f   // Noise level for speed simulation

typedef enum { MOTOR_LEFT, MOTOR_RIGHT } motor_side_t;

void motor_simulation_init(void);
void motor_simulation_update(float dt);
float motor_simulation_get_speed(motor_side_t side);
void motor_simulation_set_target_speed(motor_side_t side, float speed);

#endif  // MOTOR_SIMULATION_H
