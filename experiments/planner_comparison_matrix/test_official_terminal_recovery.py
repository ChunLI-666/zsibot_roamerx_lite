import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import recovery_navigation
SOURCE=Path(__file__).with_name('matrix_navigation.py')
function=next(n for n in ast.parse(SOURCE.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='navigation_tree')
namespace=dict(recovery_navigation=recovery_navigation)
exec(compile(ast.Module(body=[function],type_ignores=[]),str(SOURCE),'exec'),namespace)
build=namespace['navigation_tree']
class RecoveryTreeTests(unittest.TestCase):
    def test_strict_entry_has_no_unchecked_delay_retry(self):
        for variant in ['official','segmented','hybrid']:
            tree=ET.fromstring(build(variant));self.assertIsNone(tree.find('.//RecoveryNode'))
    def test_opt_in_original_goal_and_stop_release_order(self):
        config=dict(run_id='test',footprint=[[-.3,-.2],[.3,-.2],[.3,.2],[-.3,.2]],padding=.01,
            odom_topic='/odom/current_pose',scan_topic='/laser_scan',scan_max_age=.5,stop_timeout_sec=30.,health_stable_sec=1.,bt_retries=1)
        for variant in ['official','segmented']:
            tree=ET.fromstring(build(variant,config));node=tree.find('.//RecoveryNode')
            self.assertEqual(node.attrib['number_of_retries'],'1')
            sequence,repair=node
            self.assertEqual([n.tag for n in sequence],['BeginStoppedReplanAttempt','WaitForStoppedAndHealthy','ComputePathToPose','ValidateStoppedPlanStart','FollowPath'])
            self.assertEqual(sequence[2].attrib['goal'],'{goal}')
            self.assertEqual(sequence[2].attrib['use_start'],'false')
            self.assertEqual(sequence[1].attrib['scan_max_age'],'0.5')
            self.assertEqual(sequence[1].attrib['error_code_id'],'{recovery_error_code}')
            self.assertEqual(sequence[3].attrib['error_code_id'],'{recovery_error_code}')
            self.assertIsNone(tree.find('.//Delay'))
            self.assertEqual([n.tag for n in repair],['WouldAControllerRecoveryHelp','WouldAPlannerRecoveryHelp','WouldAStoppedPlanRecoveryHelp'])
if __name__=='__main__':unittest.main()
