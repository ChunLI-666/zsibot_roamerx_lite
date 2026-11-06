"""Pure actual constructor/order/early-failure tests. No ROS/Popen/socket/signal."""
import ast,copy,json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest import mock
import full_scene_mapping as o
import full_scene_mapping_contract as c
from test_v4_core import Clock,model,truth

HERE=Path(__file__).resolve().parent
class PassiveRelay(unittest.TestCase):
    def prepare(self,root):
        p=root/'root.json';p.write_text('{}');out=root/'out';out.mkdir()
        return o.OwnedRelay(out,'fixture',p,model(),{'pid':1}),p,out
    def test_real_constructor_only_file_bytes_and_close_no_child(self):
        with tempfile.TemporaryDirectory() as name,mock.patch.object(o.subprocess,'Popen',side_effect=AssertionError('no child')) as popen,mock.patch.object(o.os,'write',side_effect=AssertionError('no pipe')) as write:
            relay,p,out=self.prepare(Path(name));self.assertIsNone(relay.process);self.assertIsNone(relay.pidfd)
            self.assertEqual((out/'mapping_lateral_relay_frozen.py').read_bytes(),(HERE/'mapping_lateral_relay.py').read_bytes())
            self.assertEqual((out/'lateral_capability.py').read_bytes(),(HERE/'lateral_capability.py').read_bytes())
            admission=c.read_json(out/'relay_admission.json');self.assertEqual(admission['root_release_sha256'],c.sha(p));self.assertEqual(admission['owner_identity'],{'pid':1})
            relay.close();self.assertTrue(relay.log.closed);popen.assert_not_called();write.assert_not_called()
    def test_actual_execute_order_moves_only_passive_constructor(self):
        tree=ast.parse((HERE/'full_scene_mapping.py').read_text());execute=next(f for f in tree.body if isinstance(f,ast.FunctionDef) and f.name=='execute')
        def call(name):return next(n for n in ast.walk(execute) if isinstance(n,ast.Call) and ast.unparse(n.func)==name)
        ctor=call('OwnedRelay');init=call('rclpy.init');node=call('Node');standing=call('contract.standing_before_relay');start=call('relay.start')
        self.assertLess(ctor.lineno,init.lineno);self.assertLess(init.lineno,node.lineno);self.assertLess(node.lineno,standing.lineno);self.assertLess(standing.lineno,start.lineno)
        self.assertEqual(ast.unparse(start.args[0]),'callbacks.spin')
        fault=next(n for n in ast.walk(execute) if isinstance(n,ast.If) and ast.unparse(n.test)=='imu_gate_fault is not None' and standing.lineno<n.lineno<start.lineno)
        self.assertIsInstance(fault.body[0],ast.Raise)
        from test_v4_core import BASE as root
        base=root/'full_scene_mapping_fine_forward_profile_candidate_v10/review_source_v10/full_scene_mapping.py'
        old=ast.parse(base.read_text());old_owned=next(n for n in old.body if isinstance(n,ast.ClassDef) and n.name=='OwnedRelay');new_owned=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='OwnedRelay')
        self.assertEqual(ast.dump(old_owned,include_attributes=False),ast.dump(new_owned,include_attributes=False))
    def test_early_Node_lying_or_rawfault_failure_never_starts_or_writes_SDK(self):
        tree=ast.parse((HERE/'full_scene_mapping.py').read_text());execute=next(f for f in tree.body if isinstance(f,ast.FunctionDef) and f.name=='execute')
        standing=next(n for n in ast.walk(execute) if isinstance(n,ast.Expr) and isinstance(n.value,ast.Call) and ast.unparse(n.value.func)=='contract.standing_before_relay')
        start=next(n for n in ast.walk(execute) if isinstance(n,ast.Expr) and isinstance(n.value,ast.Call) and ast.unparse(n.value.func)=='relay.start')
        fault=next(n for n in ast.walk(execute) if isinstance(n,ast.If) and ast.unparse(n.test)=='imu_gate_fault is not None' and standing.lineno<n.lineno<start.lineno)
        for scenario in ['Node','lying','rawfault']:
            with self.subTest(scenario=scenario),tempfile.TemporaryDirectory() as name,mock.patch.object(o.subprocess,'Popen',side_effect=AssertionError('no child')) as popen,mock.patch.object(o.os,'write',side_effect=AssertionError('no SDK pipe')) as write:
                relay,p,out=self.prepare(Path(name));relay.start=mock.Mock(side_effect=AssertionError('no start'))
                clk=Clock();sample=truth(clk)
                if scenario=='lying':sample['xyz'][2]=.1
                namespace=dict(contract=c,latest={'truth':sample},model=model(),time=clk,imu_gate_fault={'reason':'fixture'} if scenario=='rawfault' else None,relay=relay,callbacks=N(spin=lambda:None))
                if scenario=='Node':
                    init_statement=next(n for n in ast.walk(execute) if isinstance(n,ast.Expr) and isinstance(n.value,ast.Call) and ast.unparse(n.value.func)=='rclpy.init')
                    node_statement=next(n for n in ast.walk(execute) if isinstance(n,ast.Assign) and isinstance(n.value,ast.Call) and ast.unparse(n.value.func)=='Node')
                    namespace.update(rclpy=N(init=mock.Mock()),SignalHandlerOptions=N(NO=0),Node=mock.Mock(side_effect=RuntimeError('fake Node failed')),release={'release_id':'fixture'})
                    with self.assertRaises(RuntimeError):exec(compile(ast.fix_missing_locations(ast.Module(body=copy.deepcopy([init_statement,node_statement]),type_ignores=[])),'<actual init+Node>','exec'),namespace)
                else:
                    with self.assertRaises(c.Hold):exec(compile(ast.fix_missing_locations(ast.Module(body=copy.deepcopy([standing,fault,start]),type_ignores=[])),'<actual startup gates>','exec'),namespace)
                result=o.finalize_owned_relay(relay,lambda:False)
                self.assertFalse(result['stop_verified']);self.assertTrue(result['relay_closed']);self.assertIsNone(result['initial_cleanup_zero_receipt']);self.assertTrue(relay.log.closed)
                relay.start.assert_not_called();popen.assert_not_called();write.assert_not_called()

if __name__=='__main__':unittest.main(verbosity=2)
