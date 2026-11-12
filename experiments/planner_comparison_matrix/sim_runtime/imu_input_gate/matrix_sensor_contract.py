#!/usr/bin/env python3
"""Explicit Matrix v1.1.2 instantaneous MID360-to-Lightning contract.

Input data is sensor-local FLU despite the source frame_id='map'.  Lidar and
IMU are co-located and parallel in the frozen mid360_slam simulator config.
This adapter preserves sampling stamps and never generates scan offsets.
"""
import copy
import numpy as np
import struct
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rclpy.serialization import serialize_message

EXPECTED = [('x',0,7), ('y',4,7), ('z',8,7), ('intensity',12,7),
            ('tag',16,2), ('line',17,2), ('timestamp',18,8)]
DTYPE = np.dtype({'names':['x','y','z','intensity','tag','line','timestamp'],
                  'formats':['<f4','<f4','<f4','<f4','u1','u1','<f8'],
                  'offsets':[0,4,8,12,16,17,18], 'itemsize':26})

def checked_points(msg):
    if msg.is_bigendian or msg.point_step != 26 or msg.height != 1:
        raise ValueError('Unexpected Matrix lidar layout')
    if [(f.name,f.offset,f.datatype) for f in msg.fields] != EXPECTED:
        raise ValueError('Unexpected Matrix lidar fields')
    pts=np.frombuffer(msg.data, dtype=DTYPE, count=msg.width)
    if not np.all(pts['timestamp'] == 0):
        raise ValueError('Nonzero per-point source time requires another contract')
    intensity=pts['intensity']
    if not np.all(np.isfinite(intensity) & (intensity >= 0) & (intensity <= 255) & (intensity == np.floor(intensity))):
        raise ValueError('Unexpected intensity, cannot losslessly map to uint8')
    return pts

def empty_custom(msg):
    out=CustomMsg()
    out.header=copy.deepcopy(msg.header)
    out.header.frame_id='livox_frame'
    out.timebase=msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
    out.lidar_id=0
    return out

def lidar_to_custom(msg):
    pts=checked_points(msg)
    out=empty_custom(msg)
    out.points=[CustomPoint(offset_time=0,x=float(p['x']),y=float(p['y']),z=float(p['z']),
                            reflectivity=int(p['intensity']),tag=int(p['tag']),line=int(p['line'])) for p in pts]
    out.point_num=len(out.points)
    return out

def lidar_to_custom_cdr(msg):
    """Vectorized CDR for the same CustomMsg; validated against ROS serializer.

    No Python point objects are created at10Hz. Prefix comes from the actual
    installed ROS type serializer; each CustomPoint has4-byte CDR alignment.
    """
    pts=checked_points(msg)
    prefix_msg=empty_custom(msg);prefix_msg.point_num=len(pts)
    prefix=bytearray(serialize_message(prefix_msg))
    if prefix[:4] != b'\x00\x01\x00\x00':
        raise ValueError('Expected little-endian CDR1')
    # Empty sequence length is the final uint32 in the serializer-generated prefix.
    prefix[-4:]=struct.pack('<I',len(pts))
    dest=np.zeros(len(pts),dtype=np.dtype({'names':['offset_time','x','y','z','reflectivity','tag','line'],
        'formats':['<u4','<f4','<f4','<f4','u1','u1','u1'],'offsets':[0,4,8,12,16,17,18],'itemsize':20}))
    for key in ('x','y','z','tag','line'):dest[key]=pts[key]
    dest['reflectivity']=pts['intensity']
    payload=dest.tobytes()
    return bytes(prefix)+(payload[:-1] if len(pts) else b'')

def imu_to_local(msg):
    out=copy.deepcopy(msg)
    out.header.frame_id='imu_link'
    return out
