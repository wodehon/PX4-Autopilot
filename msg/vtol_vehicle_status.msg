# VTOL_STATE, only used internally in the VTOL module and for setting vtol flags in vehicle_status, plus logging
uint8 VTOL_STATE_MC = 0
uint8 VTOL_STATE_FW = 1
uint8 VTOL_STATE_TRANSITION_TO_FW_P1 = 2
uint8 VTOL_STATE_TRANSITION_TO_FW_P2 = 3
uint8 VTOL_STATE_TRANSITION_TO_FW_P3 = 4
uint8 VTOL_STATE_TRANSITION_TO_FW_P4 = 5
uint8 VTOL_STATE_TRANSITION_TO_MC_P1 = 6
uint8 VTOL_STATE_TRANSITION_TO_MC_P2 = 7

uint64 timestamp			# time since system start (microseconds)

uint8 vtol_state		# current state of the vtol, see VEHICLE_VTOL_STATE

bool vtol_transition_failsafe		# vtol in transition failsafe mode
