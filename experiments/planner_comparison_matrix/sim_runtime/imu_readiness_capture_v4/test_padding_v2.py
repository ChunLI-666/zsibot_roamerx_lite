"""Padding-only regression and strict complete-wire-field negative fixtures."""
import math
import struct
import unittest
from imu_converter import imu_cdr_evidence
from test_readiness import cdr,converted,message,gid,CONFIG,RAW_IMU,IMU,Validator,c

class PaddingFixtures(unittest.TestCase):
    def test_three_alignment_bytes_are_only_exclusion_and_input_unchanged(self):
        original=cdr(100_000_000_000,'imu_link',[float(i) for i in range(37)]);before=bytes(original)
        baseline=imu_cdr_evidence(original)
        for offset in [25,26,27]:
            for value in range(256):
                changed=bytearray(original);changed[offset]=value;item=imu_cdr_evidence(changed)
                self.assertEqual(item['padding_normalized_cdr_sha256'],baseline['padding_normalized_cdr_sha256'])
        self.assertEqual(original,before)
        self.assertEqual(baseline['alignment_padding_offsets'],[25,26,27])

    def test_every_other_wire_bit_remains_significant_or_invalid(self):
        original=cdr(100_000_000_000,'imu_link',[float(i) for i in range(37)]);baseline=imu_cdr_evidence(original)['padding_normalized_cdr_sha256']
        checked=0
        for offset in range(len(original)):
            if offset in [25,26,27]:continue
            for bit in range(8):
                changed=bytearray(original);changed[offset]^=1<<bit;checked+=1
                try:item=imu_cdr_evidence(changed)
                except (ValueError,UnicodeError,struct.error):continue
                self.assertNotEqual(item['padding_normalized_cdr_sha256'],baseline,(offset,bit))
        self.assertEqual(checked,2568)

    def test_NaN_or_infinity_in_each_of_37_fields_remains_rejected(self):
        for index in range(37):
            for value in [float('nan'),float('inf'),-float('inf')]:
                values=[0.]*37;values[index]=value
                with self.subTest(field=index,value=value),self.assertRaises(ValueError):imu_cdr_evidence(cdr(100_000_000_000,'imu_link',values))

    def test_signed_zero_header_and_protocol_bits_are_not_normalized(self):
        first=imu_cdr_evidence(cdr(100_000_000_000,'imu_link'))['padding_normalized_cdr_sha256']
        values=[0.]*37;values[0]=-0.
        self.assertNotEqual(first,imu_cdr_evidence(cdr(100_000_000_000,'imu_link',values))['padding_normalized_cdr_sha256'])
        self.assertNotEqual(first,imu_cdr_evidence(cdr(100_000_000_001,'imu_link'))['padding_normalized_cdr_sha256'])
        for raw in [cdr(100_000_000_000,'map'),cdr(100_000_000_000,'imu_link')[:-1],cdr(100_000_000_000,'imu_link')+b'\0']:
            with self.assertRaises(ValueError):imu_cdr_evidence(raw)
        for offset,value in [(0,1),(1,0),(2,1),(3,1),(12,8),(16,ord('x'))]:
            changed=bytearray(cdr(100_000_000_000,'imu_link'));changed[offset]=value
            with self.assertRaises(ValueError):imu_cdr_evidence(changed)

    def test_actual_failure_shape_passes_but_finite_numeric_drift_still_HOLD(self):
        def padded_converter(raw):
            data=bytearray(converted(raw));data[25:28]=bytes.fromhex('683a20');return bytes(data)
        v=Validator(CONFIG,padded_converter);ns=100_000_000_000
        v.sample(RAW_IMU,cdr(ns),message(RAW_IMU,ns),ns,0,gid(RAW_IMU))
        v.sample(IMU,cdr(ns,'imu_link'),message(IMU,ns),ns,0,gid(IMU))
        self.assertIn(ns,v.paired);self.assertEqual(v.imu_whole_CDR_equal,0);self.assertEqual(v.imu_padding_only_differences,1)
        ns+=50_000_000;v.sample(RAW_IMU,cdr(ns),message(RAW_IMU,ns),ns,50_000_000,gid(RAW_IMU))
        values=[0.]*37;values[36]=.001
        with self.assertRaisesRegex(c.Hold,'wire fields differ outside3'):v.sample(IMU,cdr(ns,'imu_link',values),message(IMU,ns),ns,50_000_000,gid(IMU))

if __name__=='__main__':unittest.main(verbosity=2)
