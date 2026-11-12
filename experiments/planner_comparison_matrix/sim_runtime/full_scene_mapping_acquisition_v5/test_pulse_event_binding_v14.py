"""Actual execute lambda + actual helper integration; no ROS/process/socket."""
import ast,copy,unittest
from pathlib import Path
import full_scene_mapping as o
from test_fine_pulses_v13 import PulsePlant

class PulseEventBinding(unittest.TestCase):
    def test_real_execute_wrapper_preserves_route_index_and_pulse_index(self):
        tree=ast.parse(Path(o.__file__).read_text());execute=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='execute')
        expression=next(n for n in ast.walk(execute) if isinstance(n,ast.Lambda) and isinstance(n.body,ast.Call)
            and isinstance(n.body.func,ast.Name) and n.body.func.id=='emit' and n.body.args and isinstance(n.body.args[0],ast.Constant) and n.body.args[0].value=='forward_terminal_phase')
        for route_index in [0,17,4095]:
            plant=PulsePlant(ticks=2,cost=.05);events=[]
            def sink(kind,**fields):
                self.assertEqual(kind,'forward_terminal_phase');events.append(dict(fields))
                plant.emit(fields['phase'],**{k:v for k,v in fields.items() if k!='phase'})
            wrapper=eval(compile(ast.fix_missing_locations(ast.Expression(body=copy.deepcopy(expression))),'<actual execute event lambda>','eval'),dict(emit=sink,index=route_index))
            o.forward_terminal_leg(plant.start,plant.target,plant.entry,plant.p,plant.health,plant.tick,plant.stable,plant.rotate,wrapper,plant.clock)
            phases={'fine_pulse_started','fine_pulse_issued','fine_pulse_stopped'};pulse=[e for e in events if e['phase'] in phases]
            self.assertEqual({e['phase'] for e in pulse},phases);self.assertTrue(all(e['index']==route_index for e in events))
            self.assertTrue(all(type(e['pulse_index']) is int and e['pulse_index']>=1 for e in pulse))
            starts=[e['pulse_index'] for e in pulse if e['phase']=='fine_pulse_started'];stops=[e['pulse_index'] for e in pulse if e['phase']=='fine_pulse_stopped']
            self.assertEqual(starts,list(range(1,len(starts)+1)));self.assertEqual(starts,stops)
            for e in pulse:self.assertIn(e['pulse_index'],starts)
            self.assertTrue(plant.packets)

if __name__=='__main__':unittest.main(verbosity=2)
