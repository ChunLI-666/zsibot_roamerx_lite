# Header stamp is publication time, not measurement age.
std_msgs/Header header
uint16 schema_version
navigo_epoch_msgs/LocalizationIdentity identity
uint64 heartbeat_sequence
bool lifecycle_enabled
string phase
uint8 UNKNOWN=0
uint8 NORMAL=1
uint8 DEGRADED=2
uint8 LOST=3
uint8 health
# Fusion belongs to identity; output additionally requires NORMAL health.
bool fusion_ready
bool output_ready
builtin_interfaces/Time commit_source_stamp
# The actual map->navigation-base pose published for this identity, with its
# original source stamp; not a pose sampled later from an untagged TF buffer.
geometry_msgs/PoseStamped output_pose
string base_frame_id
string reason
