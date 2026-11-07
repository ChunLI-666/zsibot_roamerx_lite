import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

SOURCE=Path(__file__).with_name('matrix_navigation.py')
function=next(node for node in ast.parse(SOURCE.read_text()).body if isinstance(node,ast.FunctionDef) and node.name=='navigation_tree')
namespace={}
exec(compile(ast.Module(body=[function],type_ignores=[]),str(SOURCE),'exec'),namespace)
build=namespace['navigation_tree']
class StoppedRotationRecoveryTests(unittest.TestCase):
    def test_b_and_hybrid_keep_original_single_sequence(self):
        for variant in ['official','hybrid']:
            tree=ET.fromstring(build(variant));self.assertIsNone(tree.find('.//RecoveryNode'))
            self.assertEqual([n.tag for n in tree.find('BehaviorTree/Sequence')],['ComputePathToPose','FollowPath'])
    def test_a_recovery_recomputes_before_follow_same_goal_once(self):
        root=ET.fromstring(build('segmented'));recovery=root.find('.//RecoveryNode')
        self.assertEqual(recovery.attrib['number_of_retries'],'1')
        execute,repair=list(recovery);self.assertEqual([n.tag for n in execute],['SetBlackboard','ComputePathToPose','FollowPath'])
        self.assertEqual(execute[0].attrib,dict(output_key='follow_path_error_code',value='0'))
        self.assertEqual(execute[1].attrib['goal'],'{goal}')
        self.assertEqual(execute[2].attrib['error_code_id'],'{follow_path_error_code}')
        self.assertEqual(repair[0].tag,'WouldAControllerRecoveryHelp');self.assertEqual(repair[0].attrib['error_code'],'{follow_path_error_code}')
        self.assertEqual(repair[1].tag,'Delay');self.assertEqual(repair[1].attrib['delay_msec'],'1000')
        self.assertEqual(repair[1][0].tag,'AlwaysSuccess')
        self.assertIsNone(root.find('.//Wait'));self.assertIsNone(root.find('.//Spin'));self.assertIsNone(root.find('.//BackUp'))
if __name__=='__main__':unittest.main(verbosity=2)
