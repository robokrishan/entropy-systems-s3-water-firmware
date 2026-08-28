idf_component_register(SRCS "state_machine.c"
                    INCLUDE_DIRS "include"
                    REQUIRES state_machine_common 
                    PRIV_REQUIRES nozzle_servo)
