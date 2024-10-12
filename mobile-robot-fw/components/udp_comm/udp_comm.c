#include "udp_comm.h"

#include "esp_log.h"
#include "lwip/sockets.h"
#include "secrets.h"

#define BUFFER_SIZE 128

static int sock_receive;
static int sock_send;
static struct sockaddr_in dest_addr;

void udp_communication_init(void) {
    // Initialize UDP sockets
    // Receive socket
    sock_receive = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in recv_addr = {.sin_family = AF_INET,
                                    .sin_port = htons(PORT_RECEIVE),
                                    .sin_addr.s_addr = INADDR_ANY};

    bind(sock_receive, (struct sockaddr *)&recv_addr, sizeof(recv_addr));

    // Send socket
    sock_send = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT_SEND);
    inet_pton(AF_INET, DEST_IP,
              &dest_addr.sin_addr);  // Replace with actual IP
}

int udp_receive_command(movement_command_t *command) {
    char rx_buffer[BUFFER_SIZE];
    int len = recv(sock_receive, rx_buffer, BUFFER_SIZE - 1, 0);
    if (len > 0) {
        rx_buffer[len] = '\0';
        sscanf(rx_buffer, "%f,%f", &command->speed_magnitude,
               &command->direction_angle);
        return 1;
    }
    return 0;
}

void udp_send_telemetry(const char *data, uint16_t length) {
    sendto(sock_send, data, length, 0, (struct sockaddr *)&dest_addr,
           sizeof(dest_addr));
}
