idf_component_register(SRCS "nozzle_servo.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_mcpwm device_config)
