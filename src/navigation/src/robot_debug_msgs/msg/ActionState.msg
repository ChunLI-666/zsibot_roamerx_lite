# value is action_msgs/GoalStatus.status without added IDLE/PREEMPTED states.
string label
uint8 value
string endpoint
unique_identifier_msgs/UUID goal_id
builtin_interfaces/Time goal_accepted_stamp
builtin_interfaces/Time observed_at
BuildStamp source_build
bool source_binding_valid
