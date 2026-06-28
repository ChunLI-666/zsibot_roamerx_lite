std_msgs/Header header
uint32 seq
uint64 source_time_ms
uint32 bridge_receive_age_ms

string event
string source
string raw_payload
string command_type
string sdk_call

geometry_msgs/Twist command
geometry_msgs/Twist normalized_command
bool has_command
bool has_normalized_command

bool send_ok
bool connected
bool moving
int32 ctrl_mode
uint32 sdk_result
uint32 battery_percent
uint32 last_command_age_ms
