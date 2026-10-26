#!/usr/bin/env python3
"""Pure mocks only: never changes the test process affinity or executes a child."""
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import unittest
from unittest import mock

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'run_budget.py'
spec=importlib.util.spec_from_file_location('nav_budget_pure',SOURCE)
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)

class BudgetTests(unittest.TestCase):
 def test_default_bypass_has_no_affinity_or_audit_side_effect(self):
  command=['ros2','launch','robot_navigo','lightning_nav_bringup.launch.py','use_sim_time:=true']
  before_env=dict(os.environ);before_cwd=os.getcwd();stderr=io.StringIO()
  with mock.patch.object(b.os,'sched_getaffinity',side_effect=AssertionError('query')),mock.patch.object(b.os,'sched_setaffinity',side_effect=AssertionError('set')),mock.patch.object(b.os,'getpid',side_effect=AssertionError('pid')),mock.patch.object(b.os,'chdir',side_effect=AssertionError('cwd')),mock.patch.object(b.os,'execvp')as execute,contextlib.redirect_stderr(stderr):
   self.assertEqual(b.main(['--',*command]),0);execute.assert_called_once_with(command[0],command)
  self.assertEqual(stderr.getvalue(),'');self.assertEqual(dict(os.environ),before_env);self.assertEqual(os.getcwd(),before_cwd)

 def test_child_options_are_forwarded_verbatim(self):
  command=['ros2','launch','robot_navigo','bringup.py','--cpu-list','child-value','space value','--ros-args']
  with mock.patch.object(b.os,'sched_getaffinity',side_effect=AssertionError('query')),mock.patch.object(b.os,'execvp')as execute:
   b.main(command);execute.assert_called_once_with('ros2',command)

 def test_explicit_mask_is_set_and_read_back_before_exec(self):
  allowed=set(range(16));requested={4,5,6,7};calls=[];stderr=io.StringIO();command=['ros2','launch','robot_navigo','bringup.py']
  def get(pid):calls.append(('get',pid));return allowed if len(calls)==1 else requested
  def setmask(pid,mask):calls.append(('set',pid,set(mask)))
  def execute(executable,argv):
   self.assertEqual(calls,[('get',0),('set',0,requested),('get',0)]);self.assertEqual((executable,argv),('ros2',command));calls.append(('exec',))
  before_env=dict(os.environ);before_cwd=os.getcwd()
  with mock.patch.object(b.os,'sched_getaffinity',side_effect=get),mock.patch.object(b.os,'sched_setaffinity',side_effect=setmask),mock.patch.object(b.os,'getpid',return_value=123),mock.patch.object(b.os,'execvp',side_effect=execute),contextlib.redirect_stderr(stderr):b.main(['--cpu-list','4-7','--',*command])
  prefix='[NAV_CPU_BUDGET] ';self.assertTrue(stderr.getvalue().startswith(prefix));proof=json.loads(stderr.getvalue()[len(prefix):])
  self.assertEqual(set(proof),{'schema','enabled','pid','requested_cpus','allowed_before','actual_cpus','command'})
  self.assertEqual(proof,dict(schema=1,enabled=True,pid=123,requested_cpus=[4,5,6,7],allowed_before=list(range(16)),actual_cpus=[4,5,6,7],command=command))
  self.assertEqual(dict(os.environ),before_env);self.assertEqual(os.getcwd(),before_cwd)

 def test_general_explicit_lists_not_hardcoded_to_nav_preset(self):
  self.assertEqual(b.parse_cpu_list('2,4-6,9',{2,4,5,6,9}),{2,4,5,6,9})
  self.assertEqual(b.parse_cpu_list('0',{0,2}),{0})

 def test_invalid_or_outside_inherited_mask_rejected(self):
  for value in ('',' ','-1','4,','4,,6','4--6','6-4','4,4','4-6,6','4, 6','16','0-999999999999','4.0'):
   with self.subTest(value=value),self.assertRaises(ValueError):b.parse_cpu_list(value,set(range(16)))
  with self.assertRaises(ValueError):b.parse_cpu_list('4-6',{4,6})
  with self.assertRaises(ValueError):b.parse_cpu_list('4',set())

 def test_bad_options_or_no_command_do_not_query_affinity(self):
  with mock.patch.object(b.os,'sched_getaffinity',side_effect=AssertionError('query')),mock.patch.object(b.os,'execvp')as execute:
   for argv in ([],['--cpu-list','4-7','--'],['--cpu-list','4','--cpu-list','6','--','ros2']):
    with self.subTest(argv=argv),self.assertRaises(ValueError):b.main(argv)
   execute.assert_not_called()

 def test_readback_mismatch_stops_without_exec_or_success_log(self):
  stderr=io.StringIO()
  with mock.patch.object(b.os,'sched_getaffinity',side_effect=[set(range(16)),{4}]),mock.patch.object(b.os,'sched_setaffinity')as setter,mock.patch.object(b.os,'execvp')as execute,contextlib.redirect_stderr(stderr):
   with self.assertRaises(ValueError):b.main(['--cpu-list','4-7','--','ros2'])
   setter.assert_called_once_with(0,{4,5,6,7});execute.assert_not_called()
  self.assertEqual(stderr.getvalue(),'')

 def test_set_failure_does_not_exec(self):
  with mock.patch.object(b.os,'sched_getaffinity',return_value=set(range(16))),mock.patch.object(b.os,'sched_setaffinity',side_effect=PermissionError('denied')),mock.patch.object(b.os,'execvp')as execute:
   with self.assertRaises(PermissionError):b.main(['--cpu-list','4-7','--','ros2'])
   execute.assert_not_called()

if __name__=='__main__':unittest.main()
