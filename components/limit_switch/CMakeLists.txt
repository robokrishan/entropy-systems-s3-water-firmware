idf_component_register(SRCS "limit_switch.c"
                    INCLUDE_DIRS "include"
                    REQUIRES device_config state_machine esp_driver_gpio)
