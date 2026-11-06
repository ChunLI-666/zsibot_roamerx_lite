"""SIM-only acceptance policy. Reject old samples, never repair their clocks."""
import base64
from collections import Counter
import hashlib
import math
import struct

SCOPE='SIM_ONLY_IMU_MONOTONIC_GATE'
MAX_AGE_NS=300_000_000
FUTURE_NS=50_000_000
MAX_GAP_NS=100_000_000


def decode_imu_cdr(raw):
    """Exact sensor_msgs/Imu plain CDR header + all37 numeric values; read only."""
    data=bytes(raw)
    if len(data)<20:raise ValueError('truncated_cdr')
    representation=int.from_bytes(data[:2],'big')
    if representation not in (0,1):raise ValueError('unsupported_cdr')
    endian='<' if representation==1 else '>'
    sec,nsec,length=struct.unpack_from(endian+'iII',data,4)
    if sec<0 or nsec>=1_000_000_000 or not 1<=length<=1024:raise ValueError('invalid_header')
    end=16+length
    if end>len(data) or data[end-1]!=0:raise ValueError('invalid_frame_string')
    frame=data[16:end-1].decode('utf-8')
    aligned=4+((end-4+7)//8)*8
    if len(data)<aligned+37*8 or len(data)>aligned+37*8+3:raise ValueError('invalid_imu_cdr_size')
    values=struct.unpack_from(endian+'37d',data,aligned)
    return dict(stamp_ns=sec*1_000_000_000+nsec,frame=frame,values=values)


class ImuMonotonicGate:
    def __init__(self,expected_frame='map'):
        self.expected_frame=expected_frame
        self.last_accepted_ns=None;self.last_accepted_receipt_ns=None;self.last_accepted_receipt_steady_ns=None
        self.received=0;self.accepted=0;self.rejected=0;self.reasons=Counter();self.last_reject=None

    def inspect(self,raw,receipt_unix_ns,receipt_steady_ns,now_unix_ns=None,now_steady_ns=None):
        raw=bytes(raw);self.received+=1
        now_unix_ns=receipt_unix_ns if now_unix_ns is None else now_unix_ns
        now_steady_ns=receipt_steady_ns if now_steady_ns is None else now_steady_ns
        event=dict(scope=SCOPE,raw_index=self.received-1,cdr_sha256=hashlib.sha256(raw).hexdigest(),cdr_bytes=len(raw),
            header_ns=None,frame=None,last_accepted_header_ns=self.last_accepted_ns,
            callback_receipt_unix_ns=receipt_unix_ns,callback_receipt_steady_ns=receipt_steady_ns,
            accepted=False,reason=None,fatal=False)
        try:
            if any(type(t) is not int or t<0 for t in [receipt_unix_ns,receipt_steady_ns,now_unix_ns,now_steady_ns]):raise ValueError('invalid_receipt_clock')
            sample=decode_imu_cdr(raw);stamp=sample['stamp_ns'];event.update(header_ns=stamp,frame=sample['frame'],source_age_ns=now_unix_ns-stamp)
            if sample['frame']!=self.expected_frame:raise ValueError('changed_imu_frame')
            if not all(math.isfinite(v) for v in sample['values']):raise ValueError('nonfinite_imu')
            if self.last_accepted_ns is not None and stamp<=self.last_accepted_ns:
                event['reason']='equal_header' if stamp==self.last_accepted_ns else 'regressing_header'
            elif not -FUTURE_NS<=now_unix_ns-stamp<=MAX_AGE_NS:raise ValueError('original_header_stale_or_future')
            elif not 0<=now_steady_ns-receipt_steady_ns<=MAX_AGE_NS:raise ValueError('callback_receipt_stale_or_future')
            elif self.last_accepted_receipt_steady_ns is not None and receipt_steady_ns<self.last_accepted_receipt_steady_ns:raise ValueError('receipt_clock_regression')
            elif self.last_accepted_ns is not None and stamp-self.last_accepted_ns>MAX_GAP_NS:raise ValueError('accepted_header_gap_exceeded')
            else:
                event.update(accepted=True,reason='accepted_original_sample')
                self.last_accepted_ns=stamp;self.last_accepted_receipt_ns=receipt_unix_ns
                self.last_accepted_receipt_steady_ns=receipt_steady_ns;self.accepted+=1
        except (ValueError,UnicodeError,struct.error) as exc:event.update(reason=str(exc),fatal=True)
        if not event['accepted']:
            self.rejected+=1;self.reasons[event['reason']]+=1
            event['original_cdr_base64']=base64.b64encode(raw).decode('ascii');self.last_reject=event
        return event

    def fresh(self,now_unix_ns,now_steady_ns):
        return (self.last_accepted_ns is not None and -FUTURE_NS<=now_unix_ns-self.last_accepted_ns<=MAX_AGE_NS
                and 0<=now_steady_ns-self.last_accepted_receipt_steady_ns<=MAX_AGE_NS)

    def diagnostics(self,now_unix_ns,now_steady_ns):
        return dict(scope=SCOPE,raw_received=self.received,accepted=self.accepted,rejected=self.rejected,
            reject_reasons=dict(self.reasons),last_accepted_header_ns=self.last_accepted_ns,
            last_accepted_callback_receipt_unix_ns=self.last_accepted_receipt_ns,
            last_accepted_callback_receipt_steady_ns=self.last_accepted_receipt_steady_ns,
            accepted_input_fresh=self.fresh(now_unix_ns,now_steady_ns),no_timestamp_repair=True)
