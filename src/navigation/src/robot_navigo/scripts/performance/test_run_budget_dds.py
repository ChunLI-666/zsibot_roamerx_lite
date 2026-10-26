"""Portable fake-API DDS budget contracts; no ROS process or transport starts."""
import contextlib,io,json,os,sys,tempfile,types,unittest
from pathlib import Path
from unittest.mock import patch
from xml.etree import ElementTree
import test_run_budget as original
b=original.b
class UDPTests(unittest.TestCase):
 def setUp(self):self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
 def tearDown(self):self.tmp.cleanup()
 def env(self):return dict(ROS_DISTRO='jazzy',ROS_LOCALHOST_ONLY='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',KEEP='same')
 def identity(self):return dict(rmw_identifier_in_budget_process='rmw_fastrtps_cpp',rmw_version='8.4.4',FastDDS_version='2.14.6')
 def test_disabled_keeps_command_environment_and_skips_DDS_dependencies(self):
  for tail in [['--','ros2','launch','robot_navigo','space arg'],['--cpu-list','4,5','--','ros2','launch','robot_navigo','--native-flag']]:
   stderr=io.StringIO();command=tail[tail.index('--')+1:];before=self.env()
   with patch.dict(os.environ,before,clear=True),patch.object(b.os,'sched_getaffinity',side_effect=[set(range(16)),{4,5}]),patch.object(b.os,'sched_setaffinity'),patch.object(b.os,'getpid',return_value=123),patch.object(b.os,'execvp')as execute,patch.object(b.os,'execvpe',side_effect=AssertionError('new exec branch')),patch.object(b,'dds_udp_environment',side_effect=AssertionError('disabled metadata query')),contextlib.redirect_stderr(stderr):
    b.main(tail);execute.assert_called_once_with(command[0],command)
    self.assertEqual(dict(os.environ),before)
   if '--cpu-list' in tail:
    self.assertEqual(json.loads(stderr.getvalue().split('] ',1)[1]),dict(schema=1,enabled=True,pid=123,requested_cpus=[4,5],allowed_before=list(range(16)),actual_cpus=[4,5],command=command))
   else:self.assertEqual(stderr.getvalue(),'')
 def test_enabled_exact_delta_keeps_original_command_and_parent_env(self):
  before=self.env();stderr=io.StringIO();command=['ros2','launch','robot_navigo','bringup.py','arg:=space value']
  with patch.dict(os.environ,before,clear=True),patch.object(b,'dds_middleware_identity',return_value=self.identity()),patch.object(b.platform,'machine',return_value='x86_64'),patch.object(b.os,'sched_getaffinity',side_effect=[set(range(16)),{4,5}]),patch.object(b.os,'sched_setaffinity')as affinity,patch.object(b.os,'getpid',return_value=123),patch.object(b.os,'execvp',side_effect=AssertionError('old exec branch')),patch.object(b.os,'execvpe')as execute,contextlib.redirect_stderr(stderr):
   b.main(['--cpu-list','4,5','--dds-udp-loopback-profile','--',*command]);self.assertEqual(dict(os.environ),before)
  exe,argv,env=execute.call_args.args;self.assertEqual((exe,argv),('ros2',command));affinity.assert_called_once_with(0,{4,5})
  self.assertNotIn('ROS_LOCALHOST_ONLY',env);self.assertEqual(env['ROS_AUTOMATIC_DISCOVERY_RANGE'],'SYSTEM_DEFAULT');self.assertEqual(env['FASTRTPS_DEFAULT_PROFILES_FILE'],str(b.dds_udp_profile_path()));self.assertEqual(env['KEEP'],'same')
  lines=stderr.getvalue().splitlines();self.assertEqual(len(lines),2);cpu=json.loads(lines[0].split('] ',1)[1]);self.assertEqual(set(cpu),{'schema','enabled','pid','requested_cpus','allowed_before','actual_cpus','command'});proof=json.loads(lines[1].split('] ',1)[1]);self.assertTrue(proof['actual_native_transport_proof_required']);self.assertEqual(proof['command'],command);self.assertEqual(proof['cwd'],os.getcwd());self.assertEqual(proof['schema'],'robot_navigo.dds_udp_loopback.v1')
 def test_boolean_duplicate_or_value_rejected_before_exec(self):
  for option in [['--dds-udp-loopback-profile','--dds-udp-loopback-profile'],['--dds-udp-loopback-profile=false']]:
   with patch.object(b.os,'execvpe')as execute,contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit)as error:b.main([*option,'--','ros2'])
   self.assertEqual(error.exception.code,2);execute.assert_not_called()
 def test_conflicting_transport_variables_including_empty_rejected(self):
  for key in ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','RMW_FASTRTPS_USE_QOS_FROM_XML','RMW_FASTRTPS_PUBLICATION_MODE','ROS_STATIC_PEERS','FASTDDS_ENVIRONMENT_FILE','FASTDDS_BUILTIN_TRANSPORTS'):
   for value in ('','unexpected'):
    env=self.env();env[key]=value
    with self.subTest(key=key,value=value),self.assertRaises(ValueError):b.dds_udp_environment(env)
  env=self.env();env['RMW_IMPLEMENTATION']='rmw_cyclonedds_cpp'
  with self.assertRaises(ValueError):b.dds_udp_environment(env)
 def test_unknown_arch_distro_discovery_fail_before_affinity(self):
  for arch,distro,scope in [('aarch64','jazzy','LOCALHOST'),('x86_64','humble','LOCALHOST'),('x86_64','jazzy','SUBNET')]:
   env=self.env();env.update(ROS_DISTRO=distro,ROS_AUTOMATIC_DISCOVERY_RANGE=scope)
   with patch.dict(os.environ,env,clear=True),patch.object(b.platform,'machine',return_value=arch),patch.object(b.os,'sched_setaffinity',side_effect=AssertionError('affinity before validation')),self.assertRaises(ValueError):b.main(['--cpu-list','4,5','--dds-udp-loopback-profile','--','ros2'])
 def test_xml_changed_missing_and_cwd_implicit_forbidden(self):
  for name in ('missing','changed'):
   p=self.root/(name+'.xml')
   if name=='changed':p.write_text('<changed/>')
   with patch.object(b,'dds_udp_profile_path',return_value=p),self.assertRaises(ValueError):b.dds_udp_environment(self.env())
  for name in ('DEFAULT_FASTRTPS_PROFILES.xml','DEFAULT_FASTDDS_PROFILES.xml'):
   p=self.root/name;p.write_text('<implicit/>');cwd=os.getcwd()
   try:
    os.chdir(self.root)
    with self.assertRaises(ValueError):b.dds_udp_environment(self.env())
   finally:os.chdir(cwd);p.unlink()
 def test_install_and_source_layouts_only(self):
  for path,expected in [(self.root/'prefix/lib/robot_navigo/run_budget',self.root/'prefix/lib/robot_navigo/config/performance/ci6_udp_loopback.xml'),(self.root/'robot_navigo/scripts/performance/run_budget.py',self.root/'robot_navigo/config/performance/ci6_udp_loopback.xml')]:
   with patch.object(b,'__file__',str(path)):self.assertEqual(b.dds_udp_profile_path(),expected)
  with patch.object(b,'__file__',str(self.root/'odd/run_budget.py')),self.assertRaises(ValueError):b.dds_udp_profile_path()
 def test_exact_middleware_identity_fake_API_only(self):
  prefix=self.root/'underlay';rmw=prefix/'share/rmw_fastrtps_cpp/package.xml';rmw.parent.mkdir(parents=True);fast=prefix/'share/fastrtps/cmake/fastrtps-config-version.cmake';fast.parent.mkdir(parents=True)
  rmw.write_text('<package><version>8.4.4</version></package>');fast.write_text('set(PACKAGE_VERSION "2.14.6")\n')
  a=types.ModuleType('ament_index_python');ap=types.ModuleType('ament_index_python.packages');ap.get_package_prefix=lambda name:str(prefix);r=types.ModuleType('rclpy');ru=types.ModuleType('rclpy.utilities');ru.get_rmw_implementation_identifier=lambda:'rmw_fastrtps_cpp'
  with patch.dict(sys.modules,{'ament_index_python':a,'ament_index_python.packages':ap,'rclpy':r,'rclpy.utilities':ru}):
   self.assertEqual(b.dds_middleware_identity()['FastDDS_version'],'2.14.6')
   for mode in ('rmw','version','fast'):
    ru.get_rmw_implementation_identifier=lambda:'rmw_fastrtps_cpp';rmw.write_text('<package><version>8.4.4</version></package>');fast.write_text('set(PACKAGE_VERSION "2.14.6")\n')
    if mode=='rmw':ru.get_rmw_implementation_identifier=lambda:'rmw_cyclonedds_cpp'
    if mode=='version':rmw.write_text('<package><version>8.4.3</version></package>')
    if mode=='fast':fast.write_text('set(PACKAGE_VERSION "2.14.7")\n')
    with self.subTest(mode=mode),self.assertRaises(ValueError):b.dds_middleware_identity()
 def test_shipped_XML_is_exact_loopback_transport_without_endpoint_overrides(self):
  path=b.dds_udp_profile_path();self.assertEqual(b.file_sha(path),b.DDS_UDP_XML_SHA)
  root=ElementTree.parse(path).getroot();ns={'d':'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
  transport=root.find('d:transport_descriptors/d:transport_descriptor',ns)
  self.assertEqual(transport.findtext('d:type',namespaces=ns),'UDPv4')
  self.assertEqual([x.text for x in transport.findall('d:interfaceWhiteList/d:address',ns)],['127.0.0.1'])
  participant=root.find('d:participant',ns);self.assertEqual(participant.get('is_default_profile'),'true')
  self.assertEqual(participant.findtext('d:rtps/d:useBuiltinTransports',namespaces=ns),'false')
  self.assertEqual(participant.findtext('d:rtps/d:userTransports/d:transport_id',namespaces=ns),transport.findtext('d:transport_id',namespaces=ns))
  self.assertEqual({x.text for x in participant.findall('.//d:address',ns)},{'127.0.0.1'})
  self.assertFalse(root.findall('.//d:data_reader',ns));self.assertFalse(root.findall('.//d:data_writer',ns))
if __name__=='__main__':unittest.main()
