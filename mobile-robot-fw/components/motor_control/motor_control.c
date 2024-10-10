#include "motor_control.h"

#include "driver/mcpwm_prelude.h"
#include "esp_err.h"

#define PWM_FREQ_HZ 20000     // 20 kHz PWM frequency
#define PWM_RESOLUTION 10000  // PWM resolution
#define MOTOR_MAX_SPEED 1.0f  // Max speed (normalized between -1.0 and 1.0)

static mcpwm_cmpr_handle_t comparator_left;
static mcpwm_cmpr_handle_t comparator_right;

void motor_control_init(void) {
    // Initialize MCPWM for left and right motors
    // ... (Initialize MCPWM units, timers, operators, and comparators)
    // For brevity, initialization code is omitted
}

void motor_set_speed(motor_side_t side, float speed) {
    // Clamp speed to [-1.0, 1.0]
    if (speed > MOTOR_MAX_SPEED) speed = MOTOR_MAX_SPEED;
    if (speed < -MOTOR_MAX_SPEED) speed = -MOTOR_MAX_SPEED;

    uint32_t duty = (uint32_t)((speed + 1.0f) * (PWM_RESOLUTION / 2));

    if (side == MOTOR_LEFT) {
        // Set duty cycle for left motor
        mcpwm_comparator_set_compare_value(comparator_left, duty);
    } else {
        // Set duty cycle for right motor
        mcpwm_comparator_set_compare_value(comparator_right, duty);
    }
}
