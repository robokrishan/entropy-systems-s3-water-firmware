idf_component_register(SRCS "tests.c"
                    INCLUDE_DIRS "include"
                    REQUIRES state_machine_common state_machine nozzle_servo pump)
