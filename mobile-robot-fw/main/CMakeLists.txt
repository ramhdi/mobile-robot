idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES encoder kinematics pid telemetry udp_comm esp_timer wifi motor_simulation)
