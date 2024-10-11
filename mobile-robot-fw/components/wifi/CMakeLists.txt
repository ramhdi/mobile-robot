idf_component_register(SRCS "wifi.c"
                    INCLUDE_DIRS "."
                    REQUIRES encoder esp_event esp_wifi nvs_flash)
