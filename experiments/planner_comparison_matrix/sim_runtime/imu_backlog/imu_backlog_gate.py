"""SIM-only opt-in preservation of continuous original queued IMU samples."""
import base64
import hashlib
import importlib.util
import math
from pathlib import Path
import struct


FROZEN_SOURCE = Path('/home/charles/project/colcon_ws/analysis_outputs/planner_comparison_20261105/full_scene_private_sensor_bootstrap_v2/source')
FROZEN_GATE_SHA256 = '0b451821b94eb8182ff2f8df8d5e7faf359a24486455c90b629478f944023f66'
_gate_path = FROZEN_SOURCE/'imu_gate.py'
if hashlib.sha256(_gate_path.read_bytes()).hexdigest() != FROZEN_GATE_SHA256:
    raise ValueError('Frozen original IMU gate SHA mismatch')
_spec = importlib.util.spec_from_file_location('_imu_backlog_frozen_gate', _gate_path)
original = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(original)


def validate_imu_backlog_max_age_sec(value):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not .3 <= value <= 1.):
        raise ValueError('imu_backlog_max_age_sec must be finite in [.3,1]; default .3 retains original behavior')


class BacklogPreservingImuGate(original.ImuMonotonicGate):
    def __init__(self, expected_frame='map', imu_backlog_max_age_sec=.3):
        validate_imu_backlog_max_age_sec(imu_backlog_max_age_sec)
        super().__init__(expected_frame)
        self.imu_backlog_max_age_sec = float(imu_backlog_max_age_sec)
        self.marked_backlog_samples = 0
        self.first_marked_backlog = self.last_marked_backlog = None

    def inspect(self, raw, receipt_unix_ns, receipt_steady_ns, now_unix_ns=None, now_steady_ns=None):
        if self.imu_backlog_max_age_sec == .3:
            return super().inspect(raw,receipt_unix_ns,receipt_steady_ns,now_unix_ns,now_steady_ns)
        raw = bytes(raw)
        self.received += 1
        now_unix_ns = receipt_unix_ns if now_unix_ns is None else now_unix_ns
        now_steady_ns = receipt_steady_ns if now_steady_ns is None else now_steady_ns
        event = dict(scope=original.SCOPE,raw_index=self.received-1,
            cdr_sha256=hashlib.sha256(raw).hexdigest(),cdr_bytes=len(raw),header_ns=None,
            frame=None,last_accepted_header_ns=self.last_accepted_ns,
            callback_receipt_unix_ns=receipt_unix_ns,callback_receipt_steady_ns=receipt_steady_ns,
            accepted=False,reason=None,fatal=False)
        try:
            if any(type(t) is not int or t < 0 for t in [receipt_unix_ns,receipt_steady_ns,now_unix_ns,now_steady_ns]):
                raise ValueError('invalid_receipt_clock')
            sample = original.decode_imu_cdr(raw)
            stamp = sample['stamp_ns']
            age_ns = now_unix_ns-stamp
            event.update(header_ns=stamp,frame=sample['frame'],source_age_ns=age_ns)
            if sample['frame'] != self.expected_frame:raise ValueError('changed_imu_frame')
            if not all(math.isfinite(v) for v in sample['values']):raise ValueError('nonfinite_imu')
            if self.last_accepted_ns is not None and stamp <= self.last_accepted_ns:
                event['reason'] = 'equal_header' if stamp == self.last_accepted_ns else 'regressing_header'
            elif not -original.FUTURE_NS <= age_ns <= int(self.imu_backlog_max_age_sec*1e9):
                raise ValueError('original_header_stale_or_future')
            elif not 0 <= now_steady_ns-receipt_steady_ns <= original.MAX_AGE_NS:
                raise ValueError('callback_receipt_stale_or_future')
            elif (self.last_accepted_receipt_steady_ns is not None
                    and (receipt_steady_ns < self.last_accepted_receipt_steady_ns
                         or receipt_unix_ns < self.last_accepted_receipt_ns)):
                raise ValueError('receipt_clock_regression')
            elif self.last_accepted_ns is not None and stamp-self.last_accepted_ns > original.MAX_GAP_NS:
                # Never integrate across a real >100ms original header gap.
                raise ValueError('accepted_header_gap_exceeded')
            elif age_ns > original.MAX_AGE_NS and self.last_accepted_ns is None:
                raise ValueError('backlog_without_accepted_anchor')
            else:
                marked = age_ns > original.MAX_AGE_NS
                event.update(accepted=True,reason='accepted_original_queued_sample_with_marked_backlog'
                             if marked else 'accepted_original_sample')
                if marked:
                    self.marked_backlog_samples += 1
                    event.update(marked_backlog=True,actual_header_gap_ns=stamp-self.last_accepted_ns,
                        original_header_gap_limit_ns=original.MAX_GAP_NS,backlog_age_ns=age_ns,
                        original_age_limit_ns=original.MAX_AGE_NS,
                        configured_backlog_max_age_ns=int(self.imu_backlog_max_age_sec*1e9),
                        marked_backlog_index=self.marked_backlog_samples-1,
                        original_cdr_base64=base64.b64encode(raw).decode('ascii'),
                        strict_input_timing_status='FAIL',strict_input_timing_accepted=False,
                        no_timestamp_repair=True)
                    if self.first_marked_backlog is None:self.first_marked_backlog=event
                    self.last_marked_backlog=event
                self.last_accepted_ns=stamp
                self.last_accepted_receipt_ns=receipt_unix_ns
                self.last_accepted_receipt_steady_ns=receipt_steady_ns
                self.accepted += 1
        except (ValueError,UnicodeError,struct.error) as exc:
            event.update(reason=str(exc),fatal=True)
        if not event['accepted']:
            self.rejected += 1
            self.reasons[event['reason']] += 1
            event['original_cdr_base64']=base64.b64encode(raw).decode('ascii')
            self.last_reject=event
        return event

    # fresh() deliberately remains the original <=.3s method even in backlog mode.
    def diagnostics(self,now_unix_ns,now_steady_ns):
        result=super().diagnostics(now_unix_ns,now_steady_ns)
        result.update(imu_backlog_max_age_sec=self.imu_backlog_max_age_sec,
            original_header_gap_limit_ns=original.MAX_GAP_NS,marked_backlog_samples=self.marked_backlog_samples,
            strict_input_timing_status='FAIL' if self.marked_backlog_samples or self.rejected else 'NO_VIOLATION_RECORDED',
            strict_input_timing_accepted=False if self.marked_backlog_samples or self.rejected else None,
            first_marked_backlog_header_ns=None if self.first_marked_backlog is None else self.first_marked_backlog['header_ns'],
            last_marked_backlog_header_ns=None if self.last_marked_backlog is None else self.last_marked_backlog['header_ns'])
        return result
