idf_component_register(SRCS "diagnostics.c"
                    INCLUDE_DIRS "include"
                    REQUIRES ssd1306 ina226 state_machine)
