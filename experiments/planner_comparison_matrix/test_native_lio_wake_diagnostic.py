import ast,copy
from pathlib import Path
import unittest

SOURCE=Path(__file__).parent/'sim_runtime/imu_backlog/start_native_tracking_inputs.py'
function=next(n for n in ast.parse(SOURCE.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='lio_wake_diagnostic_profile')
def require(condition,message):
    if not condition:raise RuntimeError(message)
scope=dict(require=require)
exec(compile(ast.Module(body=[function],type_ignores=[]),str(SOURCE),'exec'),scope)
configure=scope['lio_wake_diagnostic_profile']

class NativeWakeDiagnosticTests(unittest.TestCase):
    def data(self):return dict(performance=dict(lio_imu_ready_wake=True,tbb_max_parallelism=4),odom=dict(max_stamp_delta_sec=.1,base_to_tracking_xyz=[0,0,.1]),fasterlio=dict(extrinsic_T=[0,0,0]))
    def test_default_retains_entire_template(self):
        value=self.data();before=copy.deepcopy(value);profile=configure(value)
        self.assertEqual(value,before);self.assertTrue(profile['effective_value']);self.assertFalse(profile['diagnostic_only'])
    def test_false_changes_only_explicit_scheduling_option(self):
        value=self.data();expected=copy.deepcopy(value);expected['performance']['lio_imu_ready_wake']=False
        profile=configure(value,'false');self.assertEqual(value,expected);self.assertTrue(profile['template_value']);self.assertFalse(profile['effective_value']);self.assertTrue(profile['diagnostic_only'])
    def test_invalid_mode_does_not_modify_template(self):
        value=self.data();before=copy.deepcopy(value)
        with self.assertRaises(RuntimeError):configure(value,'true')
        self.assertEqual(value,before)
    def test_false_rejects_implicit_or_nonboolean_fallback(self):
        for value in [{},dict(performance={}),dict(performance=dict(lio_imu_ready_wake='true'))]:
            before=copy.deepcopy(value)
            with self.assertRaises(RuntimeError):configure(value,'false')
            self.assertEqual(value,before)
    def test_existing_false_template_stays_false(self):
        value=self.data();value['performance']['lio_imu_ready_wake']=False;before=copy.deepcopy(value)
        profile=configure(value);self.assertEqual(value,before);self.assertFalse(profile['effective_value']);self.assertFalse(profile['diagnostic_only'])

if __name__=='__main__':unittest.main()
