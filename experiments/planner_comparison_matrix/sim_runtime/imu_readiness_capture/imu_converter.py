"""Exact baseline imu_to_local function; no CustomMsg or ROS imports."""
import copy

def imu_to_local(msg):
    out=copy.deepcopy(msg)
    out.header.frame_id='imu_link'
    return out
