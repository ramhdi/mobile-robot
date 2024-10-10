#ifndef UDP_COMMUNICATION_H
#define UDP_COMMUNICATION_H

#include <stdint.h>

typedef struct {
    float speed_magnitude;
    float direction_angle;
} movement_command_t;

void udp_communication_init(void);
int udp_receive_command(movement_command_t *command);
void udp_send_telemetry(const char *data, uint16_t length);

#endif  // UDP_COMMUNICATION_H
