navigo_epoch_msgs/LocalizationIdentity localization
# Random per BT process/context; task revision changes on goal/preempt/halt.
string navigation_session_id
uint64 task_sequence
# Increasing planning request number; retained through smoothing and control.
uint64 plan_sequence
# New safety-gate process requires a new plan, not old retained permissions.
string gate_session_id
