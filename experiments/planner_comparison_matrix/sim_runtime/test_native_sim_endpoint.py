"""Five pure tests: no ROS context, real process, socket, UDP or remote call."""
import argparse
import ast
import contextlib
import importlib.util
import io
import json
from pathlib import Path
from types import SimpleNamespace as N
import unittest
from unittest import mock

HERE=Path(__file__).resolve().parent


def load_adapter():
    created=[]
    class FakeNode:
        def __init__(self,*args):created.append('node')
        def create_publisher(self,*args):return N(publish=lambda *args:None)
        def create_subscription(self,*args):return object()
        def create_timer(self,*args):created.append('timer');return object()
        def get_logger(self):return N(info=lambda *args:None)
    tree=ast.parse((HERE/'sim_command_adapter.py').read_text())
    tree.body=[node for node in tree.body if not (
        isinstance(node,ast.Import) and any(item.name=='rclpy' for item in node.names) or
        isinstance(node,ast.ImportFrom) and node.module.split('.')[0] in {'rclpy','geometry_msgs','nav_msgs','std_msgs'})]
    namespace=dict(__file__=str(HERE/'sim_command_adapter.py'),__name__='adapter_without_ROS',Node=FakeNode,
        Twist=object,Odometry=object,UInt8=object,Bool=object,String=object)
    exec(compile(tree,str(HERE/'sim_command_adapter.py'),'exec'),namespace)
    return namespace,created


ADAPTER,CREATED=load_adapter()
SPEC=importlib.util.spec_from_file_location('relay_without_socket_open',HERE/'sim_udp_relay.py')
RELAY=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(RELAY)


class EndpointTests(unittest.TestCase):
    def test_default_loopback_and_fixed_port(self):
        args=ADAPTER['parse_args']([])
        self.assertEqual((args.relay_mode,args.sim_host),('ssh','127.0.0.1'))
        self.assertEqual(RELAY.parse_args([]).sim_host,'127.0.0.1')
        ADAPTER['require_relay_ready']('SIM_RELAY_READY 127.0.0.1:7447','127.0.0.1')
        with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):RELAY.parse_args(['--sim-port','9999'])

    def test_local_private_host_argv_and_matching_ready(self):
        args=ADAPTER['parse_args'](['--relay-mode','local','--sim-host','192.168.1.219'])
        argv=ADAPTER['relay_command'](args.relay_mode,args.sim_host)
        self.assertEqual(argv,[ADAPTER['sys'].executable,str(HERE/'sim_udp_relay.py'),'--sim-host','192.168.1.219'])
        ADAPTER['require_relay_ready']('SIM_RELAY_READY 192.168.1.219:7447',args.sim_host)
        with self.assertRaises(RuntimeError):ADAPTER['require_relay_ready']('SIM_RELAY_READY 127.0.0.1:7447',args.sim_host)

    def test_invalid_host_fails_before_node_child_or_socket(self):
        for host in ['localhost','0.0.0.0','192.168.1.219:7447','8.8.8.8']:
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):ADAPTER['parse_args'](['--relay-mode','local','--sim-host',host])
            with mock.patch.object(RELAY.socket,'socket',side_effect=AssertionError('socket forbidden')) as opened:
                with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):RELAY.main(['--sim-host',host])
                opened.assert_not_called()
        before=list(CREATED)
        with mock.patch.object(ADAPTER['subprocess'],'Popen',side_effect=AssertionError('child forbidden')):
            with self.assertRaises(ValueError):ADAPTER['Adapter'](N(relay_mode='local',sim_host='8.8.8.8'))
        self.assertEqual(CREATED,before)

    def test_ssh_legacy_argv_unchanged_and_nonloopback_rejected(self):
        self.assertEqual(ADAPTER['relay_command']('ssh'),['ssh','-o','BatchMode=yes','-o','ConnectTimeout=5','-o','ServerAliveInterval=2','-o','ServerAliveCountMax=2','charles@192.168.1.219','python3','/mnt/data/planner_comparison_20261105/sim_udp_relay.py'])
        with self.assertRaises(ValueError):ADAPTER['relay_command']('ssh','192.168.1.219')
        with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):ADAPTER['parse_args'](['--sim-host','192.168.1.219'])

    def test_ready_mismatch_closes_owned_child_and_relay_eof_sends_cleanup_zeros(self):
        process=N(stdin=io.StringIO(),stdout=io.StringIO('SIM_RELAY_READY 127.0.0.1:7447\n'),wait=mock.Mock(return_value=0),terminate=mock.Mock())
        with mock.patch.object(ADAPTER['subprocess'],'Popen',return_value=process):
            with self.assertRaises(RuntimeError):ADAPTER['Adapter'](N(relay_mode='local',sim_host='192.168.1.219',max_policy_vx=.4,max_policy_wz=.3))
        self.assertTrue(process.stdin.closed);process.wait.assert_called_once_with(timeout=3)
        self.assertNotIn('timer',CREATED)
        sock=mock.Mock();output=io.StringIO();stream=io.StringIO('')
        with mock.patch.object(RELAY.socket,'socket',return_value=sock),mock.patch.object(RELAY.sys,'stdin',stream),\
             mock.patch.object(RELAY.select,'select',return_value=([stream],[],[])),mock.patch.object(RELAY.time,'sleep'),contextlib.redirect_stdout(output):
            RELAY.main(['--sim-host','192.168.1.219'])
        self.assertEqual(output.getvalue().splitlines()[0],'SIM_RELAY_READY 192.168.1.219:7447')
        self.assertEqual(sock.sendto.call_count,5)
        for call in sock.sendto.call_args_list:
            packet=json.loads(call.args[0]);self.assertEqual(call.args[1],('192.168.1.219',7447))
            self.assertEqual([packet[key] for key in ['forward','lateral','yaw']],[0.,0.,0.])
        sock.close.assert_called_once()


if __name__=='__main__':unittest.main()
