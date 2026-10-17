# Exactly one bounded payload is set. No Path/Costmap/cloud is copied.
std_msgs/Header header
BuildStamp source_build
BuildStamp reporter_build
string source_instance
string component
string endpoint
bool goal_id_valid
unique_identifier_msgs/UUID goal_id
uint64 path_revision
uint64 dropped_records
CheckerSample[<=1] checker
BranchSample[<=1] branch
GateSample[<=1] gate
TransportSample[<=1] transport
ExecutionSample[<=1] execution
