"""Exact baseline imu_to_local function; no CustomMsg or ROS imports."""
import copy
import hashlib
import math
from imu_gate import decode_imu_cdr

def imu_to_local(msg):
    out=copy.deepcopy(msg)
    out.header.frame_id='imu_link'
    return out

def imu_cdr_evidence(raw):
    """Keep every wire bit except the3 CDR1 alignment bytes after imu_link NUL.

    Fixed deployed schema: CDR1 little-endian/options0, header.frame_id=imu_link,
    string end25, next double aligned to payload-relative8 => byte28, then37
    contiguous float64 fields. No header, value, trailing or unknown byte is
    excluded. Original bytes and original SHA remain untouched evidence.
    """
    raw=bytes(raw);sample=decode_imu_cdr(raw)
    if (len(raw)!=324 or raw[:4]!=b'\x00\x01\x00\x00'
        or raw[12:25]!=b'\x09\x00\x00\x00imu_link\x00'
        or sample['frame']!='imu_link' or not all(math.isfinite(v) for v in sample['values'])):
        raise ValueError('unsupported/nonfinite derived IMU comparison layout')
    normalized=raw[:25]+b'\x00\x00\x00'+raw[28:]
    return dict(schema='sensor_msgs.Imu.cdr1_le.imu_link.padding_only.v1',wire_bytes=len(raw),
        original_cdr_sha256=hashlib.sha256(raw).hexdigest(),alignment_padding_offsets=[25,26,27],
        alignment_padding_hex=raw[25:28].hex(),padding_normalized_cdr_sha256=hashlib.sha256(normalized).hexdigest(),
        all37_numeric_field_bits_preserved=True)
