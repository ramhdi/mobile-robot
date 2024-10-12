#include "telemetry.h"

#include <stdio.h>

#include "esp_log.h"
#include "kinematics.h"
#include "udp_comm.h"

void telemetry_init(void) {
    // Initialize UDP communication for telemetry
    udp_communication_init();
}

void telemetry_send(void) {
    pose_t pose = kinematics_get_pose();
    float linear_speed = kinematics_get_linear_speed();
    float angular_speed = kinematics_get_angular_speed();

    char telemetry_data[128];
    int len = snprintf(telemetry_data, sizeof(telemetry_data),
                       "Position: (%.2f, %.2f, %.2f), Speed: (%.2f, %.2f)",
                       pose.x, pose.y, pose.theta, linear_speed, angular_speed);

    udp_send_telemetry(telemetry_data, len);
    ESP_LOGI("TELEMETRY", "%s", telemetry_data);
}
