navigo_epoch_msgs/LocalizationIdentity localization
# Random per BT process/context; task revision changes on goal/preempt/halt.
string navigation_session_id
uint64 task_sequence
# Increasing motion request number (historical field name); planning or explicit BACKUP.
# Retained through smoothing and control; never reused when changing action kind.
uint64 plan_sequence
# New safety-gate process requires a new plan, not old retained permissions.
string gate_session_id
# Explicit motion authority. TRACK remains the default for existing clients.
uint8 TRACK=0
uint8 BACKUP=1
uint8 execution_kind
# BACKUP only. Immutable across renewal; a new action needs a new token.
float64 backup_max_speed
uint64 execution_deadline_ns
