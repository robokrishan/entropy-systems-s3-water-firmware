idf_component_register(SRCS "state_machine_global_events.c" "state_machine_state_init.c" "state_machine_states.c"
                        "state_machine_state_stowed.c" "state_machine_state_lowering.c"
                        "state_machine_state_deployed.c" "state_machine_state_pumping.c"
                        "state_machine_state_raising.c" "state_machine_state_pos_unknown.c"
                        "state_machine_state_fault.c" "state_machine_global_events.c"
                    INCLUDE_DIRS "include"
                    REQUIRES state_machine_common state_machine
                    PRIV_REQUIRES nozzle_servo pump motion_timeout)
