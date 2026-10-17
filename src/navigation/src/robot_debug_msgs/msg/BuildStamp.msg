# A build fact supplied by the actual target; never inferred from runtime HEAD.
uint8 UNKNOWN=0
uint8 CLEAN=1
uint8 DIRTY=2
string repository_id
string git_commit
string artifact_id
string build_id
uint8 dirty_state
bool known
