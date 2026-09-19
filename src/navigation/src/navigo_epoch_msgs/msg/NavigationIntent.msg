# BT task authority, renewed only while its matching FollowPath is live.
navigo_epoch_msgs/NavigationToken token
uint64 heartbeat_sequence
bool active
# Navigation producers and gate must share this Linux boot / monotonic clock.
string boot_id
uint64 source_steady_time_ns
