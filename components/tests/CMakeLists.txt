idf_component_register(SRCS "test_diagnostics.c" "test_helpers.c" "test_ssd1306.c" "test_ina226.c" "test_rc_integration.c" "test_rc_input.c" "test_motion_timeout.c" "test_limit_switch.c" "test_pump.c" "test_nozzle_servo.c" "test_state_machine.c"
                    INCLUDE_DIRS "include"
                    REQUIRES state_machine_common 
                            state_machine nozzle_servo pump
                            limit_switch motion_timeout ina226
                            i2c ssd1306)
