# A read model. Republication does not refresh original observation times.
std_msgs/Header header
BuildStamp reporter_build
ActionState[<=64] actions
StatusObservation[<=32] observations
bool current_checker_valid
string current_checker_reason
StatusObservation[<=1] current_checker
uint64 observer_dropped_records
