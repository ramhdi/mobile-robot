idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES encoder kinematics motor_control pid telemetry udp_comm esp_timer wifi)
