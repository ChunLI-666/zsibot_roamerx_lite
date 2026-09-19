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
