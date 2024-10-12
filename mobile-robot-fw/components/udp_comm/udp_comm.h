#ifndef UDP_COMM_H
#define UDP_COMM_H

#include <stdint.h>

typedef struct {
    float speed_magnitude;
    float direction_angle;
} movement_command_t;

#define PORT_RECEIVE 5000
#define PORT_SEND 12345

void udp_communication_init(void);
int udp_receive_command(movement_command_t *command);
void udp_send_telemetry(const char *data, uint16_t length);

#endif  // UDP_COMM_H
