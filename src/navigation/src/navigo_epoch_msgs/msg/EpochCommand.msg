navigo_epoch_msgs/NavigationToken token
string controller_session_id
uint64 command_sequence
# Must survive smoothing unchanged. Repeated interpolation cannot renew age.
string boot_id
uint64 source_steady_time_ns
uint32 max_age_ms
# Relay identity and sequence distinguish reordered interpolation outputs.
string relay_session_id
uint64 relay_sequence
geometry_msgs/Twist velocity
uint8 PHASE_UNSPECIFIED=0
uint8 PHASE_HOLD=1
uint8 PHASE_TRANSLATE=2
uint8 PHASE_ROTATE=3
# MPPI forward tracking may contain coupled vx and wz predicted together.
uint8 PHASE_TRACK=4
uint8 source_motion_phase
# Relay-only: braking an old axis before starting the source motion phase.
bool transition_braking
# Relay-only: phase that produced the old velocity currently being braked.
uint8 braking_from_phase
