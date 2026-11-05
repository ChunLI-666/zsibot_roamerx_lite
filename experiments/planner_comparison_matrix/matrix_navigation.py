#!/usr/bin/env python3
"""Prepare/start only navigation and evidence capture against an already healthy Matrix+Lightning stack.

No fake plant, localization status publisher, TF publisher, sensor bridge, SDK adapter or goal client.
"""
import argparse,copy,hashlib,json,math,os,pathlib,shutil,signal,subprocess,sys,time
import yaml
from static_layer_snapshot import EXCLUDED_LAYERS,capture as capture_static_layers,audit_recorded as audit_static_layers
BASE=pathlib.Path(__file__).resolve().parent
NAV=BASE.parents[1]
def sha(path):
    with pathlib.Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()
def write(path,value):
    path=pathlib.Path(path);temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value,indent=2)+'\n');temporary.replace(path)

def source_snapshot(path,template):
    """Validate a deployment source manifest without inspecting a Git checkout."""
    def unique_object(pairs):
        result={}
        for key,value in pairs:
            if key in result:raise ValueError('Duplicate source snapshot key: '+key)
            result[key]=value
        return result
    def hex_digest(value,length):
        return isinstance(value,str) and len(value)==length and all(c in '0123456789abcdef' for c in value)
    raw=pathlib.Path(path).read_bytes()
    data=json.loads(raw,object_pairs_hook=unique_object)
    if not isinstance(data,dict) or set(data)!={'schema_version','origin','files_sha256'}:
        raise ValueError('Source snapshot requires schema_version, origin and files_sha256')
    if type(data['schema_version']) is not int or data['schema_version']!=1:
        raise ValueError('Unsupported source snapshot schema_version')
    origin=data['origin']
    if not isinstance(origin,dict) or set(origin)!={'kind','canonical_repository','commit','branch','status'}:
        raise ValueError('Source snapshot origin requires canonical Git observation')
    if origin['kind']!='canonical_checkout_snapshot' or not hex_digest(origin['commit'],40):
        raise ValueError('Invalid source snapshot origin kind or commit')
    if not isinstance(origin['canonical_repository'],str) or not pathlib.PurePosixPath(origin['canonical_repository']).is_absolute():
        raise ValueError('Source snapshot canonical_repository must be an absolute origin path')
    if not isinstance(origin['branch'],str) or not origin['branch'] or not isinstance(origin['status'],str):
        raise ValueError('Source snapshot branch and status must preserve the canonical observation')
    files=data['files_sha256']
    if not isinstance(files,dict) or not files:raise ValueError('Source snapshot requires source file hashes')
    root=NAV.resolve()
    # The wrapper executes before this process; the other files are consumed here.
    required=[pathlib.Path(__file__),BASE/'static_layer_snapshot.py',BASE/'run_matrix_navigation.sh',
      template,NAV/'src/navigation/src/robot_navigo/scripts/nav_safety_gate.py']
    required_names=set()
    for source in required:
        resolved=source.resolve(strict=True)
        if not resolved.is_relative_to(root):raise ValueError('Source input escapes deployment NAV: '+str(source))
        required_names.add(source.absolute().relative_to(root).as_posix())
    if not required_names<=files.keys():
        raise ValueError('Source snapshot missing required files: '+repr(sorted(required_names-files.keys())))
    for name,digest in files.items():
        relative=pathlib.PurePosixPath(name)
        if not name or relative.is_absolute() or '..' in relative.parts or '\\' in name or relative.as_posix()!=name:
            raise ValueError('Source snapshot file must be a normalized relative NAV path: '+name)
        resolved=(root/name).resolve(strict=True)
        if not resolved.is_relative_to(root) or not resolved.is_file():
            raise ValueError('Source snapshot file escapes deployment NAV or is not a file: '+name)
        if not hex_digest(digest,64) or sha(resolved)!=digest:raise ValueError('Source snapshot hash mismatch: '+name)
    return data,raw

def prepare(args):
    template=pathlib.Path(args.template).absolute()
    snapshot,snapshot_raw=source_snapshot(args.source_snapshot,template) if args.source_snapshot else (None,None)
    out=pathlib.Path(args.output).resolve();out.mkdir(parents=True,exist_ok=False)
    footprint=json.loads(args.footprint)
    if len(footprint)<3 or any(len(p)!=2 or any(not math.isfinite(v) for v in p) for p in footprint):raise ValueError('finite polygon required')
    crosses=[]
    for a,b,c in zip(footprint,footprint[1:]+footprint[:1],footprint[2:]+footprint[:2]):
        crosses.append((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))
    if not (all(x>0 for x in crosses) or all(x<0 for x in crosses)):raise ValueError('strictly convex footprint required')
    if not 0<=args.padding<=.2:raise ValueError('invalid padding')
    if not 0<args.xy_tolerance<=.25 or not 0<args.yaw_tolerance<=.25:raise ValueError('invalid goal tolerance')
    map_yaml=pathlib.Path(args.map).resolve();map_data=yaml.safe_load(map_yaml.read_text())
    if abs(float(map_data['resolution'])-.01)>1e-8:raise ValueError('this comparison requires 1cm geometry map')
    map_image=(map_yaml.parent/map_data['image']).resolve()
    template=template.resolve()
    d=yaml.safe_load(template.read_text());rpp=copy.deepcopy(d['controller_server']['ros__parameters']['FollowPath'].get('rpp',d['controller_server']['ros__parameters']['FollowPath']))
    rpp.pop('plugin',None)
    if args.rotate_threshold is not None:rpp['rotate_to_heading_min_angle']=args.rotate_threshold
    selected_path=args.segmented_install if args.variant=='segmented' else args.official_install
    if not selected_path:raise ValueError('Provide an explicit --segmented-install or --official-install for the selected variant')
    selected_install=pathlib.Path(selected_path).resolve()
    required_plugin={'official':'official_lattice_rpp_adapter::CheckedLattice','hybrid':'official_lattice_rpp_adapter::CheckedHybrid','segmented':'robot_feasible_navigation::SafeLatticePlanner'}[args.variant]
    manifest=dict(scope='Matrix navigation experiment only; not physical robot acceptance',stage='prepared',execution_requested=args.run,variant=args.variant,
        selected_install=str(selected_install),map_yaml=str(map_yaml),map_yaml_sha256=sha(map_yaml),map_image=str(map_image),map_image_sha256=sha(map_image),
        profile_template=str(template),profile_template_sha256=sha(template),footprint=footprint,padding=args.padding,
        rotate_to_heading_min_angle=rpp['rotate_to_heading_min_angle'],
        odom_topic=args.odom_topic,scan_topic=args.scan_topic,camera_topic=args.camera_topic,camera_info_topic=args.camera_info_topic,
        global_costmap=dict(rolling_window=True,width_m=20,height_m=20,resolution=.01,publish_frequency=.5),
        local_costmap=dict(rolling_window=True,width_m=6,height_m=6,resolution=.01,publish_frequency=1.),
        goal_tolerances=dict(xy=args.xy_tolerance,yaw=args.yaw_tolerance,stateful=False),goal_distance_from_start_limit_m=8.,
        command_chain=['/cmd_vel_nav','/cmd_vel','/cmd_vel_safe'],goal_client='Separate authorized agent must record UUID, original goal, result/cancel and settled errors',
        build_receipt=str(pathlib.Path(args.build_receipt).resolve()) if args.build_receipt else None,
        source_repository_observation=None if snapshot else dict(head=subprocess.check_output(['git','-C',str(NAV),'rev-parse','HEAD'],text=True).strip(),
          status=subprocess.check_output(['git','-C',str(NAV),'status','--short'],text=True)),
        caution='Repository observation is not a substitute for the selected artifact build receipt')
    if snapshot:
        frozen=out/'deployment_source_snapshot.json';frozen.write_bytes(snapshot_raw)
        manifest['deployment_source_snapshot']=dict(path=str(frozen),sha256=sha(frozen),origin=snapshot['origin'],
          validated_files_sha256=snapshot['files_sha256'],deployment_nav=str(NAV),
          note='Supplied canonical source observation and verified deployment bytes; no deployment Git checkout was inspected. This does not replace the selected artifact build receipt.')
    if args.build_receipt:
        receipt=pathlib.Path(args.build_receipt).resolve();shutil.copy2(receipt,out/'source_build_receipt.json');manifest['build_receipt_sha256']=sha(receipt)
    for node in ['controller_server','velocity_smoother','bt_navigator']:d[node]['ros__parameters']['odom_topic']=args.odom_topic
    d['map_server']['ros__parameters']['yaml_filename']=str(map_yaml)
    for name,size,frequency in [('global_costmap',20,.5),('local_costmap',6,1.)]:
        p=d[name][name]['ros__parameters'];p.update(footprint=json.dumps(footprint),footprint_padding=args.padding,
          rolling_window=True,width=size,height=size,resolution=.01,publish_frequency=frequency,
          update_frequency=2. if name=='global_costmap' else 10.,always_send_full_costmap=True)
        p['obstacle_layer']=dict(plugin='nav2_costmap_2d::ObstacleLayer',enabled=True,footprint_clearing_enabled=False,observation_sources='scan',
          scan=dict(topic=args.scan_topic,data_type='LaserScan',marking=True,clearing=True,inf_is_valid=False,
            observation_persistence=0.,expected_update_rate=.3,min_obstacle_height=0.,max_obstacle_height=2.,
            raytrace_max_range=8.5,raytrace_min_range=.0,obstacle_max_range=8.,obstacle_min_range=.1))
    planner=d['planner_server']['ros__parameters']['GridBased'];planner.update(plugin=required_plugin,tolerance=0.,max_planning_time=2.,rotation_penalty=1.)
    controller=d['controller_server']['ros__parameters'];controller['Checker'].update(stateful=False,xy_goal_tolerance=args.xy_tolerance,yaw_goal_tolerance=args.yaw_tolerance,controller_name='FollowPath')
    if args.variant=='segmented':
        planner['lattice_filepath']=str(selected_install/'share/robot_feasible_navigation/params/stop_turn_1cm_h16.json')
        controller['Checker']['plugin']='nav2_controller::SimpleGoalChecker'
        controller['FollowPath']=dict(rpp,plugin='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController')
        controller['segments']=dict(xy_tolerance=args.xy_tolerance,yaw_tolerance=args.yaw_tolerance,stopped_linear_velocity=.01,stopped_angular_velocity=.01,
          same_xy_epsilon=.0001,progress_search_distance=.25,odom_timeout=.3,rotation_anchor_wait_timeout=15.,command_sweep_horizon=1.)
    else:
        controller['Checker']['plugin']='official_lattice_rpp_adapter::FeasibleGoalChecker'
        controller['FollowPath']=dict(plugin='official_lattice_rpp_adapter::FeasibleRPP',rpp=rpp,odom_timeout=.3,sweep_horizon=1.,
          completion_path_length=max(.08,args.xy_tolerance),allow_static_path_transform=False,handler=dict(enforce_path_rotation=True,enforce_path_inversion=True,
          inversion_xy_tolerance=args.xy_tolerance,inversion_yaw_tolerance=args.yaw_tolerance,minimum_rotation_angle=.05,prune_distance=2.,max_robot_pose_search_dist=3.))
        if args.variant=='hybrid':
            planner.pop('lattice_filepath',None);planner.update(motion_model_for_search='DUBIN',angle_quantization_bins=72,
              minimum_turning_radius=.5,analytic_expansion_max_length=3.,analytic_expansion_ratio=3.5)
            manifest['control_only']='Hybrid is auxiliary control; primary comparison is segmented versus official Lattice.'
        else:planner['lattice_filepath']=str(selected_install/'share/official_lattice_rpp_adapter/params/stop_turn_1cm_h16_small_radius.json')
    # Freeze the actual primitive input instead of depending on subsequent canonical branch switches.
    if 'lattice_filepath' in planner:
        source=pathlib.Path(planner['lattice_filepath']);shutil.copy2(source,out/'lattice.json');planner['lattice_filepath']=str(out/'lattice.json');manifest['lattice_sha256']=sha(out/'lattice.json')
    bt=out/'navigate.xml';bt.write_text('<root BTCPP_format="4" main_tree_to_execute="MainTree"><BehaviorTree ID="MainTree"><Sequence><ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/><FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="Checker" progress_checker_id="Progress"/></Sequence></BehaviorTree></root>\n')
    d['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=str(bt)
    params=out/'params.yaml';params.write_text(yaml.safe_dump(d,sort_keys=False));manifest['params_sha256']=sha(params)
    shutil.copy2(__file__,out/'launcher_snapshot.py');manifest['launcher_sha256']=sha(__file__)
    helper=BASE/'static_layer_snapshot.py';shutil.copy2(helper,out/helper.name)
    manifest['static_layer_snapshot_helper_sha256']=sha(helper)
    manifest['recording_policy']=dict(excluded_continuous_topics_regex=EXCLUDED_LAYERS,
      replacement='One original-CDR snapshot of each frozen static layer under /experiment/static_layer_snapshot; map and master costmaps remain continuous')
    write(out/'runtime_manifest.json',manifest);write(out/'invocation.json',dict(argv=sys.argv,cwd=os.getcwd()))
    return out,selected_install,manifest

def run(args,out,selected_install,manifest):
    if os.environ.get('ROS_DOMAIN_ID')!='185':raise RuntimeError('Matrix navigation requires the isolated domain185 environment from simulation_setup')
    if not args.build_receipt:raise RuntimeError('--run requires a concrete selected-artifact --build-receipt')
    if not args.message_install:raise RuntimeError('--run requires an isolated custom-message --message-install for complete recording')
    if args.source_snapshot:
        # Recheck the frozen manifest immediately before runtime setup, including
        # files copied later (safety gate); a preparation result is not a lease.
        frozen=out/'deployment_source_snapshot.json'
        if sha(frozen)!=manifest['deployment_source_snapshot']['sha256']:
            raise RuntimeError('Deployment source snapshot changed after preparation')
        source_snapshot(frozen,pathlib.Path(args.template).absolute())
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from rclpy.serialization import serialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CompressedImage,CameraInfo,LaserScan
    from std_msgs.msg import UInt8,Bool
    from nav2_msgs.msg import Costmap
    from nav2_msgs.srv import GetCostmap
    from lifecycle_msgs.srv import GetState
    from rcl_interfaces.srv import ListParameters,GetParameters
    from tf2_ros import Buffer,TransformListener
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy,DurabilityPolicy
    import cv2,numpy as np
    stopped=False;static_snapshots=[]
    def request_stop(*unused):
        nonlocal stopped;stopped=True
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,request_stop);signal.signal(signal.SIGTERM,request_stop)
    node=Node('matrix_navigation_evidence_observer');latest={};counts={};procs=[];logs=[]
    event_stream=(out/'events.jsonl').open('w')
    def event(kind,**kw):
        event_stream.write(json.dumps(dict(time_ns=node.get_clock().now().nanoseconds,steady=time.monotonic(),kind=kind,**kw))+'\n');event_stream.flush()
    def receive(topic,msg):latest[topic]=(time.monotonic(),msg);counts[topic]=counts.get(topic,0)+1
    subscriptions=[]
    costmap_evidence_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for topic,kind in [(args.odom_topic,Odometry),('/lightning/loc_status',UInt8),('/lightning/pose_valid',Bool),(args.scan_topic,LaserScan),(args.camera_topic,CompressedImage),(args.camera_info_topic,CameraInfo),
                       ('/global_costmap/costmap_raw',Costmap),('/local_costmap/costmap_raw',Costmap)]:
        qos=costmap_evidence_qos if topic in ('/global_costmap/costmap_raw','/local_costmap/costmap_raw') else qos_profile_sensor_data
        subscriptions.append(node.create_subscription(kind,topic,lambda m,t=topic:receive(t,m),qos))
    tf=Buffer();listener=TransformListener(tf,node)
    def source_health():
        now=node.get_clock().now().nanoseconds/1e9
        for topic,limit in [(args.odom_topic,.3),(args.scan_topic,.3),('/lightning/loc_status',.2),('/lightning/pose_valid',.2),(args.camera_topic,.5)]:
            if topic not in latest:raise RuntimeError('Missing source '+topic)
            received,msg=latest[topic]
            if time.monotonic()-received>limit:raise RuntimeError('Stale receive '+topic)
            if hasattr(msg,'header'):
                age=now-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
                if not -.05<=age<=limit:raise RuntimeError('Stale/future stamp '+topic)
        if latest['/lightning/loc_status'][1].data!=1 or not latest['/lightning/pose_valid'][1].data:
            raise RuntimeError('Localization is not NORMAL with valid pose')
        for target,source in [('odom','base_link'),('map','odom')]:
            transform=tf.lookup_transform(target,source,Time());age=now-transform.header.stamp.sec-transform.header.stamp.nanosec/1e9
            if not -.05<=age<=.3:raise RuntimeError('Stale/future actual TF '+source+' -> '+target)
    def pump(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end and rclpy.ok():rclpy.spin_once(node,timeout_sec=.02)
    def wait(future,timeout=3.,operation=None):
        started=time.monotonic();end=started+timeout;outcome=None
        def rpc_event(kind,**kw):
            if operation:event(kind,operation=operation,elapsed_sec=time.monotonic()-started,timeout_sec=timeout,**kw)
        rpc_event('rpc_begin')
        try:
            while not future.done() and time.monotonic()<end and not stopped:rclpy.spin_once(node,timeout_sec=.02)
            if not future.done():
                future.cancel();outcome='rpc_stopped' if stopped else 'rpc_timeout'
                rpc_event(outcome,cancel_requested=True)
                raise TimeoutError((operation or 'service response')+(' stopped' if stopped else ' timeout'))
            response=future.result()
        except Exception as e:
            if not future.done():future.cancel()
            if operation and outcome is None:
                rpc_event('rpc_error',error=repr(e))
                raise RuntimeError(operation+': '+repr(e)) from e
            raise
        rpc_event('rpc_complete')
        return response
    runtime=out/'runtime_install';shutil.copytree(selected_install,runtime)
    receipt=json.loads((out/'source_build_receipt.json').read_text())
    commit=receipt.get('repository_commit','')
    if len(commit)!=40 or any(c not in '0123456789abcdef' for c in commit):raise RuntimeError('Build receipt requires full repository_commit SHA')
    if receipt.get('source_files_match_commit') is not True:raise RuntimeError('Build receipt must prove selected source files match its commit')
    declared=receipt.get('artifact_sha256',{})
    required=['lib/libsegment_controller_core.so','lib/libsafe_lattice_planner.so','lib/robot_feasible_navigation/segment_controller_server'] if args.variant=='segmented' else ['lib/libofficial_lattice_rpp_adapter.so']
    for rel in required:
        if rel not in declared:raise RuntimeError('Missing required artifact in build receipt: '+rel)
    for rel,digest in declared.items():
        artifact=(runtime/rel).resolve()
        if not artifact.is_relative_to(runtime.resolve()) or not artifact.is_file() or sha(artifact)!=digest:raise RuntimeError('Selected runtime does not match concrete build receipt: '+rel)
    env=os.environ.copy();env['AMENT_PREFIX_PATH']=str(runtime)+':/opt/ros/jazzy';env['LD_LIBRARY_PATH']=str(runtime/'lib')+':/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu'
    message_runtime=out/'message_runtime';shutil.copytree(pathlib.Path(args.message_install).resolve(),message_runtime)
    recorder_env=env.copy()
    recorder_env['AMENT_PREFIX_PATH']=str(message_runtime)+':'+env['AMENT_PREFIX_PATH']
    recorder_env['LD_LIBRARY_PATH']=str(message_runtime/'lib')+':'+env['LD_LIBRARY_PATH']
    recorder_env['PYTHONPATH']=str(message_runtime/'lib/python3.12/site-packages')+':'+env.get('PYTHONPATH','')
    recorder_env['PYTHONDONTWRITEBYTECODE']='1'
    recorder_name='matrix_recorder_'+str(os.getpid());manifest['recorder_node_name']=recorder_name
    custom_types=['livox_ros_driver2/msg/CustomMsg','robots_dog_msgs/msg/LivoxBridgeDebug','robot_debug_msgs/msg/BuildInfo','lightning/msg/LightningDebug']
    verify_code='from rosidl_runtime_py.utilities import get_message\nfrom rclpy.type_support import check_for_type_support\nimport sys\nfor name in sys.argv[1:]:\n cls=get_message(name);check_for_type_support(cls);print(name)\n'
    checked=subprocess.run(['/usr/bin/python3','-c',verify_code,*custom_types],env=recorder_env,text=True,capture_output=True,check=True)
    manifest['message_runtime']=dict(source=str(pathlib.Path(args.message_install).resolve()),required_types=custom_types,
      import_check_stdout=checked.stdout,files_sha256={str(p.relative_to(message_runtime)):sha(p) for p in message_runtime.rglob('*') if p.is_file()})
    plugins='\n'.join(p.read_text() for p in (runtime/'share').rglob('*.xml') if 'plugin' in p.name)
    if manifest['variant']=='hybrid' and 'official_lattice_rpp_adapter::CheckedHybrid' not in plugins:raise RuntimeError('CheckedHybrid not installed; refusing an unguarded fallback')
    manifest['environment']={k:env.get(k) for k in ['ROS_DOMAIN_ID','ROS_AUTOMATIC_DISCOVERY_RANGE','RMW_IMPLEMENTATION','FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','AMENT_PREFIX_PATH','LD_LIBRARY_PATH']}
    manifest['runtime_install_files']={str(p.relative_to(runtime)):sha(p) for p in runtime.rglob('*') if p.is_file()}
    write(out/'runtime_manifest.json',manifest)
    def launch(name,cmd,process_env=None):
        log=(out/(name+'.log')).open('w');logs.append(log);proc=subprocess.Popen(cmd,env=env if process_env is None else process_env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);procs.append((name,proc));event('process_started',name=name,pid=proc.pid,argv=cmd)
    try:
        pump(2.)
        for topic in ['/cmd_vel_nav','/cmd_vel','/cmd_vel_safe']:
            if node.get_publishers_info_by_topic(topic):raise RuntimeError('Existing velocity publisher requires owner cleanup before this run: '+topic)
        # Recorder covers preflight source data, all lifecycle transitions, full maps, camera and actions.
        launch('recorder',['/opt/ros/jazzy/bin/ros2','bag','record','--node-name',recorder_name,'-a','--include-hidden-topics','--exclude-regex',EXCLUDED_LAYERS,'-s','mcap','--storage-preset-profile','zstd_fast','--max-cache-size','268435456','-o',str(out/'bag')],recorder_env)
        deadline=time.monotonic()+args.preflight_timeout;preflight={}
        while time.monotonic()<deadline and not stopped:
            pump(.1)
            try:
                now=node.get_clock().now().nanoseconds/1e9
                for topic in [args.odom_topic,args.camera_topic,args.camera_info_topic,'/lightning/loc_status']:
                    if topic not in latest:raise ValueError('missing '+topic)
                    received,msg=latest[topic]
                    receipt_limit=.2 if topic=='/lightning/loc_status' else .3 if topic==args.odom_topic else .5
                    if topic!=args.camera_info_topic and time.monotonic()-received>receipt_limit:raise ValueError('stale receive '+topic)
                    if hasattr(msg,'header') and topic!=args.camera_info_topic:
                        age=now-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
                        if not -.05<=age<=(.3 if topic==args.odom_topic else .5):raise ValueError('stale/future header '+topic)
                if latest['/lightning/loc_status'][1].data!=1:raise ValueError('localization is not NORMAL')
                for target,source in [('odom','base_link'),('map','odom')]:
                    t=tf.lookup_transform(target,source,Time());age=now-t.header.stamp.sec-t.header.stamp.nanosec/1e9
                    if not -.05<=age<=.3:raise ValueError('stale/future actual TF '+source+' -> '+target)
                if counts.get(args.odom_topic,0)<3 or counts.get(args.camera_topic,0)<3:raise ValueError('waiting for source samples')
                image=latest[args.camera_topic][1];decoded=cv2.imdecode(np.frombuffer(bytes(image.data),np.uint8),cv2.IMREAD_COLOR)
                if decoded is None:raise ValueError('front camera cannot decode')
                info=latest[args.camera_info_topic][1]
                if decoded.shape[1]!=info.width or decoded.shape[0]!=info.height or info.k[0]<=0 or info.k[4]<=0:raise ValueError('camera calibration does not match decoded frame')
                (out/'front_camera_first.jpg').write_bytes(bytes(image.data));write(out/'front_camera_info.json',message_to_ordereddict(info))
                preflight=dict(ok=True,counts=counts.copy(),decoded_camera_size=[decoded.shape[1],decoded.shape[0]],note='Observed real sources; this observer publishes no sensor/status/TF data')
                break
            except Exception as e:preflight=dict(ok=False,reason=str(e))
        write(out/'preflight.json',preflight)
        if not preflight.get('ok'):raise RuntimeError('preflight failed: '+str(preflight))
        gate=out/'nav_safety_gate.py';shutil.copy2(NAV/'src/navigation/src/robot_navigo/scripts/nav_safety_gate.py',gate)
        manifest['gate_source_sha256']=sha(gate);write(out/'runtime_manifest.json',manifest)
        common=['--ros-args','--params-file',str(out/'params.yaml')]
        launch('nav_safety_gate',['/usr/bin/python3',str(gate)]+common)
        for package,exe,name,remaps in [('nav2_map_server','map_server','map_server',[]),('nav2_planner','planner_server','planner_server',[]),
          ('nav2_controller','controller_server','controller_server',['-r','cmd_vel:=cmd_vel_nav']),
          ('nav2_velocity_smoother','velocity_smoother','velocity_smoother',['-r','cmd_vel:=cmd_vel_nav','-r','cmd_vel_smoothed:=cmd_vel']),
          ('nav2_bt_navigator','bt_navigator','bt_navigator',[]),('nav2_lifecycle_manager','lifecycle_manager','lifecycle_manager',[])]:
            exe_path=str(runtime/'lib/robot_feasible_navigation/segment_controller_server') if name=='controller_server' and args.variant=='segmented' else '/opt/ros/jazzy/lib/'+package+'/'+exe
            launch(name,[exe_path]+common+remaps)
        write(out/'processes.json',{name:p.pid for name,p in procs})
        for name in ['map_server','planner_server','controller_server','velocity_smoother','bt_navigator']:
            client=node.create_client(GetState,'/'+name+'/get_state');deadline=time.monotonic()+90
            while time.monotonic()<deadline and not stopped:
                pump(.1)
                if client.service_is_ready() and wait(client.call_async(GetState.Request())).current_state.id==3:break
            else:raise RuntimeError(name+' did not activate')
        params={}
        for name in ['map_server','planner_server','controller_server','local_costmap/local_costmap','global_costmap/global_costmap','velocity_smoother','bt_navigator','nav_safety_gate']:
            lc=node.create_client(ListParameters,'/'+name+'/list_parameters');gc=node.create_client(GetParameters,'/'+name+'/get_parameters')
            if not lc.wait_for_service(timeout_sec=2.):raise RuntimeError('parameter service unavailable '+name)
            names=wait(lc.call_async(ListParameters.Request()),timeout=10.,operation='/'+name+'/list_parameters').result.names;req=GetParameters.Request();req.names=names
            params[name]={n:message_to_ordereddict(v) for n,v in zip(names,wait(gc.call_async(req),timeout=10.,operation='/'+name+'/get_parameters').values)}
        write(out/'runtime_parameters.json',params)
        static_snapshots=capture_static_layers(node,out,recorder_name,pump,lambda:stopped)
        actual_maps={}
        for name,size in [('global_costmap',2000),('local_costmap',600)]:
            client=node.create_client(GetCostmap,'/'+name+'/get_costmap')
            if not client.wait_for_service(timeout_sec=3.):raise RuntimeError('missing actual costmap service')
            msg=wait(client.call_async(GetCostmap.Request())).map
            if abs(msg.metadata.resolution-.01)>1e-6 or msg.metadata.size_x!=size or msg.metadata.size_y!=size:raise RuntimeError('actual rolling geometry mismatch '+name)
            (out/(name+'_initial.cdr')).write_bytes(serialize_message(msg));actual_maps[name]=message_to_ordereddict(msg.metadata)
        deadline=time.monotonic()+6.
        while time.monotonic()<deadline and any(counts.get('/'+n+'/costmap_raw',0)==0 for n in ['global_costmap','local_costmap']):pump(.1)
        if any(counts.get('/'+n+'/costmap_raw',0)==0 for n in ['global_costmap','local_costmap']):raise RuntimeError('full raw costmap publication not observed')
        identities={}
        for name,p in procs:
            maps=pathlib.Path(f'/proc/{p.pid}/maps').read_text()
            libraries=set(line.split()[-1] for line in maps.splitlines() if len(line.split())>=6 and
              ('/opt/ros/jazzy/lib/' in line or str(runtime/'lib') in line))
            identities[name]=dict(pid=p.pid,cmdline=pathlib.Path(f'/proc/{p.pid}/cmdline').read_bytes().replace(b'\0',b' ').decode(),maps=maps,
              selected_libraries_sha256={library:sha(library) for library in libraries if pathlib.Path(library).is_file()})
        write(out/'runtime_identities.json',identities)
        recording_topics=[args.camera_topic,args.camera_info_topic,args.odom_topic,args.scan_topic,
          '/livox/lidar','/livox/imu','/lightning_bridge/debug','/lightning/debug','/debug/build_info',
          '/tf','/tf_static','/global_costmap/costmap_raw','/local_costmap/costmap_raw']
        recording_subscriptions={topic:[info.node_name for info in node.get_subscriptions_info_by_topic(topic)] for topic in recording_topics}
        write(out/'recording_subscriptions.json',recording_subscriptions)
        missing=[topic for topic,names in recording_subscriptions.items() if recorder_name not in names]
        if missing:raise RuntimeError('Recorder subscriptions missing before READY: '+repr(missing))
        # Startup may take time. Recheck the current source values immediately before
        # authorizing a separate goal pilot, rather than reusing an earlier preflight.
        pump(.2);ready_now=node.get_clock().now().nanoseconds/1e9
        for topic in [args.odom_topic,'/lightning/loc_status',args.camera_topic,args.camera_info_topic]:
            if topic not in latest:raise RuntimeError('Source disappeared before ready: '+topic)
            received,msg=latest[topic];limit=.2 if topic=='/lightning/loc_status' else .3 if topic==args.odom_topic else .5
            if topic!=args.camera_info_topic and time.monotonic()-received>limit:raise RuntimeError('Stale source before ready: '+topic)
            if topic in [args.odom_topic,args.camera_topic]:
                age=ready_now-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
                if not -.05<=age<=limit:raise RuntimeError('Stale/future source stamp before ready: '+topic)
        if latest['/lightning/loc_status'][1].data!=1:raise RuntimeError('Localization left NORMAL before ready')
        for target,source in [('odom','base_link'),('map','odom')]:
            transform=tf.lookup_transform(target,source,Time());age=ready_now-transform.header.stamp.sec-transform.header.stamp.nanosec/1e9
            if not -.05<=age<=.3:raise RuntimeError('Stale/future actual TF before ready: '+source+' -> '+target)
        camera=latest[args.camera_topic][1];info=latest[args.camera_info_topic][1]
        decoded=cv2.imdecode(np.frombuffer(bytes(camera.data),np.uint8),cv2.IMREAD_COLOR)
        if decoded is None or decoded.shape[1]!=info.width or decoded.shape[0]!=info.height:raise RuntimeError('Front camera invalid before ready')
        source_health()
        readiness=dict(ready=True,time_ns=node.get_clock().now().nanoseconds,variant=args.variant,actual_costmaps=actual_maps,
          params_sha256=sha(out/'params.yaml'),build_receipt_sha256=manifest['build_receipt_sha256'],source_counts=counts,
          action_server='/navigate_to_pose',goal_tolerances=manifest['goal_tolerances'],goal_client_authorization='Wait for root instruction; this file alone does not dispatch navigation')
        readiness['valid_until_ns']=node.get_clock().now().nanoseconds+500_000_000
        write(out/'ready.json',readiness)
        event('navigation_ready');print(str(out/'ready.json'),flush=True)
        while not stopped:
            pump(.1)
            dead=[(n,p.returncode) for n,p in procs if p.poll() is not None]
            if dead:raise RuntimeError('owned process exited '+repr(dead))
            previous=readiness['ready']
            try:source_health();readiness.update(ready=True,reason=None)
            except Exception as e:readiness.update(ready=False,reason=str(e))
            readiness.update(time_ns=node.get_clock().now().nanoseconds,valid_until_ns=node.get_clock().now().nanoseconds+500_000_000)
            write(out/'ready.json',readiness)
            if readiness['ready']!=previous:event('readiness_changed',ready=readiness['ready'],reason=readiness['reason'])
    except Exception as e:write(out/'failed.json',dict(error=repr(e)));event('failure',error=repr(e));raise
    finally:
        readiness=json.loads((out/'ready.json').read_text()) if (out/'ready.json').exists() else {}
        readiness.update(ready=False,stopped_at_ns=node.get_clock().now().nanoseconds)
        write(out/'ready.json',readiness)
        # Stop command-producing servers first; keep smoother/gate/recorder alive for the zero tail.
        for name,p in procs:
            if name in ['bt_navigator','controller_server'] and p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        pump(1.)
        for name,p in reversed(procs):
            if name!='recorder' and p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        pump(.5)
        for name,p in procs:
            if name=='recorder' and p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for name,p in procs:
            try:p.wait(timeout=30. if name=='recorder' else 8.)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=3.)
        for log in logs:log.close()
        metadata=out/'bag/metadata.yaml'
        if metadata.exists():
            md=yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information'];topic_counts={x['topic_metadata']['name']:x['message_count'] for x in md['topics_with_message_count']}
            try:snapshot_audit=audit_static_layers(out/'bag',static_snapshots)
            except Exception as error:snapshot_audit=dict(passed=False,error=repr(error))
            write(out/'static_layer_recording_audit.json',snapshot_audit)
            required=[args.camera_topic,args.camera_info_topic,args.odom_topic,args.scan_topic,'/livox/lidar','/livox/imu','/lightning_bridge/debug','/lightning/debug','/debug/build_info','/tf','/tf_static','/map','/global_costmap/costmap_raw','/local_costmap/costmap_raw','/cmd_vel_nav','/cmd_vel','/cmd_vel_safe','/simulation/command_adapter/policy_cmd']
            required += [snapshot['recorded_topic'] for snapshot in static_snapshots]
            write(out/'final_audit.json',dict(message_count=md['message_count'],topic_counts=topic_counts,required_topics_nonempty={t:topic_counts.get(t,0)>0 for t in required},
              static_layer_exactly_once_cdr_verified=snapshot_audit['passed'],mcap_sha256={p.name:sha(p) for p in (out/'bag').glob('*.mcap')},note='Sequential decode and per-case action/results remain required before acceptance.'))
        event('stopped',observed_counts=counts);event_stream.close();node.destroy_node();rclpy.shutdown()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map',required=True);p.add_argument('--footprint',required=True,help='JSON list of body-frame [x,y] vertices')
    p.add_argument('--padding',required=True,type=float);p.add_argument('--variant',required=True,choices=['official','segmented','hybrid']);p.add_argument('--output',required=True)
    p.add_argument('--odom-topic',default='/odom/current_pose');p.add_argument('--scan-topic',default='/laser_scan');p.add_argument('--camera-topic',default='/camera/front/image/compressed');p.add_argument('--camera-info-topic',default='/camera/front/camera_info')
    p.add_argument('--template',default=str(BASE/'matrix_profile.yaml'))
    p.add_argument('--source-snapshot',help='Validated source manifest for a Git-free frozen deployment; defaults to observing canonical Git')
    p.add_argument('--official-install');p.add_argument('--segmented-install')
    p.add_argument('--rotate-threshold',type=float,help='Explicit RPP heading threshold override; otherwise frozen template value')
    p.add_argument('--xy-tolerance',type=float,default=.15);p.add_argument('--yaw-tolerance',type=float,default=.08)
    p.add_argument('--build-receipt');p.add_argument('--message-install');p.add_argument('--preflight-timeout',type=float,default=30.);p.add_argument('--run',action='store_true')
    args=p.parse_args();out,install,manifest=prepare(args)
    if args.run:
        try:run(args,out,install,manifest)
        except Exception as error:
            if not (out/'failed.json').exists():write(out/'failed.json',dict(error=repr(error),stage='receipt_or_runtime_setup'))
            write(out/'ready.json',dict(ready=False,stopped_at_ns=time.time_ns()))
            raise
    else:print(str(out/'runtime_manifest.json'))
if __name__=='__main__':main()
