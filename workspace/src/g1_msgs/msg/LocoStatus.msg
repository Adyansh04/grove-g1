# Locomotion-authority values for the `authority` field below -- see g1_locomotion's
# README for the full acquire/release state machine these correspond to.
uint8 RELEASED=0
uint8 ACQUIRING=1
uint8 HELD=2
uint8 RELEASING=3

builtin_interfaces/Time stamp
int32 fsm_id            # -1 = unknown: no SetLocoMode goal has completed successfully yet
uint8 authority
int32 last_error_code
uint32 ignored_cmd_vel  # cumulative non-zero ~/cmd_vel samples discarded for lack of authority
