"""Exercise actual graph-gate methods with endpoint mocks; no ROS/data/TF."""
import argparse
import ast
import json
from pathlib import Path
import sys
from types import MethodType, SimpleNamespace

import pytest
import yaml


HERE = Path(__file__).parent


def actual_methods():
    tree = ast.parse((HERE / 'observer.py').read_text())
    monitor = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Monitor')
    names = {'preplay_consumer_requirements', 'preplay_consumers_ready'}
    selected = [n for n in tree.body if isinstance(n, ast.FunctionDef) and
                n.name in {'recorder_endpoint_evidence', 'preplay_endpoint_matches',
                           'preplay_component_names', 'preplay_component_response_matches'}]
    selected += [n for n in monitor.body if isinstance(n, ast.FunctionDef) and n.name in names]
    counter = iter(range(100, 10000))
    scope = dict(time=SimpleNamespace(monotonic_ns=lambda: next(counter),
        process_time_ns=lambda: next(counter)), write_json=lambda p, d: Path(p).write_text(json.dumps(d)),
        preplay_list_nodes_type=lambda: SimpleNamespace(Request=SimpleNamespace), GetState=object())
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(HERE / 'observer.py'), 'exec'), scope)
    return scope


def info(requirement, **changes):
    qos = {k: SimpleNamespace(name=requirement[k]) for k in ('reliability', 'durability')}
    qos.update(history=SimpleNamespace(name='KEEP_LAST'), depth=requirement['depth'])
    value = SimpleNamespace(node_name=requirement['node_name'], node_namespace='/',
        topic_type=requirement['topic_type'], endpoint_type=SimpleNamespace(name=requirement['endpoint_type']),
        endpoint_gid=bytes(range(24)), qos_profile=SimpleNamespace(**qos))
    for key, item in changes.items():
        setattr(value, key, item)
    return value


def monitor(tmp_path, best_effort=False, arm_runtime=False):
    (tmp_path / 'lightning_config.yaml').write_text(yaml.safe_dump(dict(
        common=dict(qos_best_effort=best_effort), performance=dict(imu_queue_depth=2000, lidar_queue_depth=10))))
    (tmp_path / 'navigation_params.yaml').write_text('nav_safety_gate:\n  ros__parameters:\n    enable_epoch_contract: false\n')
    clock = SimpleNamespace(value=1000)
    obj = SimpleNamespace(args=SimpleNamespace(output=tmp_path, ready_timeout=2., full_bag_mode=True,
                                               arm_runtime=arm_runtime),
        plant=False, events=[], get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=clock.value)))
    methods = actual_methods()
    for key in ('preplay_consumer_requirements', 'preplay_consumers_ready'):
        setattr(obj, key, MethodType(methods[key], obj))
    obj.record = lambda name, data: obj.events.append((name, data.copy()))
    sensors = ['/livox/lidar', '/livox/imu']
    types = dict(zip(sensors, ('livox_ros_driver2/msg/CustomMsg', 'sensor_msgs/msg/Imu')))
    required = obj.preplay_consumer_requirements(sensors, types)
    graph = {(r['topic'], r['endpoint_type']): [info(r)] for r in required}
    # Localization and projector share a topic; preserve both endpoint owners.
    for r in required:
        key = r['topic'], r['endpoint_type']
        if not any(x.node_name == r['node_name'] for x in graph[key]): graph[key].append(info(r))
    obj.get_subscriptions_info_by_topic = lambda topic: graph.get((topic, 'SUBSCRIPTION'), [])
    obj.get_publishers_info_by_topic = lambda topic: graph.get((topic, 'PUBLISHER'), [])
    component_names = methods['preplay_component_names'](arm_runtime)
    obj.component_responses = [SimpleNamespace(full_node_names=component_names,
                                              unique_ids=list(range(1, len(component_names) + 1)))]
    obj.component_requests = []
    obj.service_graph = {
        'navigo_container': [('/navigo_container/_container/list_nodes', ['composition_interfaces/srv/ListNodes'])],
        'smoother_server': [('/smoother_server/get_state', ['lifecycle_msgs/srv/GetState'])]}
    obj.get_service_names_and_types_by_node = lambda name, namespace: obj.service_graph.get(name, [])
    def create_client(kind, service):
        def call(request):
            assert service == '/navigo_container/_container/list_nodes'
            obj.component_requests.append(request)
            response = obj.component_responses.pop(0) if len(obj.component_responses)>1 else obj.component_responses[0]
            return SimpleNamespace(done=lambda: True, result=lambda: response)
        return SimpleNamespace(service_is_ready=lambda: True, call_async=call)
    obj.create_client = create_client
    def spin(predicate, seconds, label, processes):
        assert seconds == 2.
        if not predicate(): raise TimeoutError(label)
    obj.spin_until = spin
    return obj, graph, required, sensors, types, clock


def run(obj, sensors, types):
    obj.preplay_consumers_ready(object(), sensors, types, 1000)


def test_ready_paused_graph_without_any_good_tf_scan_or_new_clock(tmp_path):
    obj, graph, required, sensors, types, clock = monitor(tmp_path)
    assert not hasattr(obj, 'latest') and not hasattr(obj, 'tf_buffer')
    run(obj, sensors, types)
    d = json.loads((tmp_path / 'preplay_consumers.json').read_text())
    assert d['status'] == 'PASS' and d['paused_clock_unchanged']
    assert len(d['requirements']) == 9 and all(x['matches'] for x in d['endpoints'])
    assert d['waits_for_good_tf_or_sensor_data'] is False
    assert d['source_resume_called_by_gate'] is False and clock.value == 1000
    assert d['observer_process_cpu_wait_ns'] >= 0 and d['wait_monotonic_ns'] >= 0
    assert all(x['matches'][0]['endpoint_gid'] == list(range(24)) for x in d['endpoints'])
    assert d['construction_required'] and d['component_container']['matches']
    assert len(d['component_container']['unique_ids']) == 8
    assert d['standalone_smoother_service']['service_ready']
    assert d['standalone_smoother_service']['state_requested'] is False


def test_arm_runtime_accepts_exact_seven_components_without_waypoint(tmp_path):
    obj, _, _, sensors, types, _ = monitor(tmp_path, arm_runtime=True)
    run(obj, sensors, types)
    report = json.loads((tmp_path / 'preplay_consumers.json').read_text())
    component = report['component_container']
    assert report['status'] == 'PASS' and component['matches']
    assert len(component['unique_ids']) == 7
    assert '/waypoint_follower' not in component['expected_full_node_names']


@pytest.mark.parametrize('arm_runtime', [False, True])
def test_component_profile_mismatch_is_rejected(tmp_path, arm_runtime):
    obj, _, _, sensors, types, _ = monitor(tmp_path, arm_runtime=arm_runtime)
    other_names = actual_methods()['preplay_component_names'](not arm_runtime)
    obj.component_responses = [SimpleNamespace(full_node_names=other_names,
                                               unique_ids=list(range(1, len(other_names) + 1)))]
    with pytest.raises(TimeoutError):
        run(obj, sensors, types)
    report = json.loads((tmp_path / 'preplay_consumers.json').read_text())
    assert report['status'] == 'FAIL' and not report['component_container']['matches']


def test_real_fastdds_16_byte_graph_gid_is_preserved(tmp_path):
    obj, graph, required, sensors, types, clock = monitor(tmp_path)
    # Representation observed in the failed on40s run; no padding fabricated.
    actual = [1, 15, 206, 185, 24, 176, 51, 37, 0, 0, 0, 0, 0, 0, 24, 4]
    for endpoints in graph.values():
        for endpoint in endpoints: endpoint.endpoint_gid = actual.copy()
    run(obj, sensors, types)
    d = json.loads((tmp_path / 'preplay_consumers.json').read_text())
    assert d['status'] == 'PASS' and d['paused_clock_unchanged']
    assert all(x['matches'][0]['endpoint_gid'] == actual for x in d['endpoints'])


@pytest.mark.parametrize('gid', [[], [1]*15, [1]*17, [1]*23, [1]*25,
    [0]*16, [0]*24, [-1]+[1]*15, [256]+[1]*15, ['1']+[1]*15,
    [True]+[1]*15, [1.0]+[1]*15])
def test_invalid_gid_length_byte_or_zero_never_satisfies_graph(tmp_path, gid):
    obj, graph, required, sensors, types, clock = monitor(tmp_path)
    graph[('/cmd_vel', 'SUBSCRIPTION')][0].endpoint_gid = gid
    with pytest.raises(TimeoutError): run(obj, sensors, types)
    d = json.loads((tmp_path / 'preplay_consumers.json').read_text())
    assert d['status'] == 'FAIL'
    assert not next(x for x in d['endpoints'] if x['requirement']['topic'] == '/cmd_vel')['matches']


@pytest.mark.parametrize('fault', ['node', 'namespace', 'type', 'direction', 'reliability', 'durability', 'depth', 'history', 'gid'])
def test_wrong_named_or_typed_or_qos_endpoint_cannot_satisfy_gate(tmp_path, fault):
    obj, graph, required, sensors, types, clock = monitor(tmp_path)
    endpoint = graph[('/cmd_vel', 'SUBSCRIPTION')][0]
    if fault == 'node': endpoint.node_name = 'rosbag2_recorder'
    elif fault == 'namespace': endpoint.node_namespace = '/other'
    elif fault == 'type': endpoint.topic_type = 'std_msgs/msg/String'
    elif fault == 'direction': endpoint.endpoint_type.name = 'PUBLISHER'
    elif fault == 'reliability': endpoint.qos_profile.reliability.name = 'BEST_EFFORT'
    elif fault == 'durability': endpoint.qos_profile.durability.name = 'TRANSIENT_LOCAL'
    elif fault == 'depth': endpoint.qos_profile.depth = 99
    elif fault == 'history': endpoint.qos_profile.history.name = 'KEEP_ALL'
    elif fault == 'gid': endpoint.endpoint_gid = bytes(24)
    with pytest.raises(TimeoutError): run(obj, sensors, types)
    d = json.loads((tmp_path / 'preplay_consumers.json').read_text())
    assert d['status'] == 'FAIL' and d['paused_clock_unchanged']
    assert any(not row['matches'] for row in d['endpoints'])
    assert d['endpoints'] and obj.events[-1][0] == 'preplay_consumers_not_ready'


def test_discovery_delay_waits_on_predicate_without_sleep(tmp_path):
    obj, graph, required, sensors, types, clock = monitor(tmp_path)
    missing = graph.pop(('/laser_scan', 'PUBLISHER'))
    def spin(predicate, *args):
        assert not predicate()
        graph[('/laser_scan', 'PUBLISHER')] = missing
        assert predicate()
    obj.spin_until = spin
    run(obj, sensors, types)
    assert obj.events[-1][0] == 'preplay_consumers_ready'


def test_unadvertised_history_depth_recorded_without_faking_values(tmp_path):
    obj, graph, required, sensors, types, clock = monitor(tmp_path)
    endpoint = graph[('/cmd_vel', 'SUBSCRIPTION')][0]
    endpoint.qos_profile.history.name, endpoint.qos_profile.depth = 'UNKNOWN', 0
    run(obj, sensors, types)
    d = json.loads((tmp_path / 'preplay_consumers.json').read_text())
    row = next(x for x in d['endpoints'] if x['requirement']['topic'] == '/cmd_vel')
    assert row['matches'][0]['qos']['history'] == 'UNKNOWN'
    assert row['matches'][0]['qos']['depth'] == 0


@pytest.mark.parametrize('before', [0, 999, 1001])
def test_clock_must_remain_original_paused_start(tmp_path, before):
    obj, graph, required, sensors, types, clock = monitor(tmp_path)
    clock.value = before
    with pytest.raises(RuntimeError, match='Paused source clock'): run(obj, sensors, types)
    assert json.loads((tmp_path / 'preplay_consumers.json').read_text())['status'] == 'FAIL'


def test_clock_advancement_during_discovery_fails(tmp_path):
    obj, graph, required, sensors, types, clock = monitor(tmp_path)
    def spin(predicate, *args):
        clock.value += 1
        predicate()
    obj.spin_until = spin
    with pytest.raises(RuntimeError, match='clock advanced'): run(obj, sensors, types)
    d = json.loads((tmp_path / 'preplay_consumers.json').read_text())
    assert d['status'] == 'FAIL' and not d['paused_clock_unchanged']


def test_clock_advancement_after_successful_predicate_still_fails(tmp_path):
    obj, graph, required, sensors, types, clock = monitor(tmp_path)
    def spin(predicate, *args):
        assert predicate()
        clock.value += 1
    obj.spin_until = spin
    with pytest.raises(RuntimeError, match='clock advanced'): run(obj, sensors, types)
    assert json.loads((tmp_path / 'preplay_consumers.json').read_text())['status'] == 'FAIL'


def test_epoch_gate_contract_is_not_mistaken_for_standalone(tmp_path):
    obj, graph, required, sensors, types, clock = monitor(tmp_path)
    (tmp_path / 'navigation_params.yaml').write_text('nav_safety_gate:\n  ros__parameters:\n    enable_epoch_contract: true\n')
    with pytest.raises(ValueError, match='standalone safety'): run(obj, sensors, types)


def test_native_best_effort_and_depth_follow_effective_yaml(tmp_path):
    obj, graph, required, sensors, types, clock = monitor(tmp_path, best_effort=True)
    native = [r for r in required if r['node_name'] == 'lightning_slam']
    assert [r['depth'] for r in native] == [10, 2000]
    assert all(r['reliability'] == 'BEST_EFFORT' for r in native)
    run(obj, sensors, types)


@pytest.mark.parametrize('full,plant', [(False, False), (True, True)])
def test_only_full_real_sensor_mode(tmp_path, full, plant):
    obj, graph, required, sensors, types, clock = monitor(tmp_path)
    obj.args.full_bag_mode, obj.plant = full, plant
    with pytest.raises(ValueError, match='full real-sensor'): run(obj, sensors, types)


def test_flag_default_off_and_fullbag_validation_before_bootstrap_io():
    tree = ast.parse((HERE / 'common.py').read_text())
    selected = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in ('arguments', 'bootstrap')]
    scope = dict(argparse=argparse, Path=Path, NAV_SOURCE=HERE, require_default_environment=lambda: None)
    exec(compile(ast.Module(body=selected, type_ignores=[]), '<actual common>', 'exec'), scope)
    parser = scope['arguments']('test')
    a = parser.parse_args(['--install-prefix', '/nonexistent', '--output', '/nonexistent'])
    assert a.require_preplay_consumers is False
    assert a.cleanup_diagnostics is False
    a.require_preplay_consumers = True
    with pytest.raises(ValueError, match='requires --full-bag-mode'): scope['bootstrap'](a)


@pytest.mark.parametrize('entry', ['replay.py', 'closed_loop.py'])
@pytest.mark.parametrize('enabled', [False, True])
def test_both_actual_entry_parsers_have_one_cleanup_flag_default_false(entry, enabled, monkeypatch):
    common = ast.parse((HERE / 'common.py').read_text())
    parser_fn = next(n for n in common.body if isinstance(n, ast.FunctionDef) and n.name == 'arguments')
    scope = dict(argparse=argparse, Path=Path, NAV_SOURCE=HERE, __doc__='test')
    exec(compile(ast.Module(body=[parser_fn], type_ignores=[]), '<actual arguments>', 'exec'),scope)
    tree=ast.parse((HERE / entry).read_text())
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    statements=[]
    for statement in main.body:
        statements.append(statement)
        if isinstance(statement,ast.Assign) and any(isinstance(n,ast.Call) and
            isinstance(n.func,ast.Attribute) and n.func.attr=='parse_args' for n in ast.walk(statement)):
            break
    argv=[entry,'--install-prefix','/nonexistent','--output','/nonexistent']
    if entry=='replay.py':argv+=['--bag','/nonexistent']
    if enabled:argv+=['--cleanup-diagnostics']
    monkeypatch.setattr(sys,'argv',argv)
    exec(compile(ast.Module(body=statements,type_ignores=[]),'<actual '+entry+'>','exec'),scope)
    assert scope['args'].cleanup_diagnostics is enabled


@pytest.mark.parametrize('fault', ['missing', 'extra', 'duplicate_name', 'wrong_name',
    'duplicate_id', 'zero_id', 'overflow_id', 'bool_id', 'length'])
def test_listnodes_requires_all_eight_exact_names_and_unique_uint64_ids(tmp_path, fault):
    obj, graph, required, sensors, types, clock=monitor(tmp_path)
    r=obj.component_responses[0]
    if fault=='missing':r.full_node_names.pop();r.unique_ids.pop()
    elif fault=='extra':r.full_node_names.append('/extra');r.unique_ids.append(9)
    elif fault=='duplicate_name':r.full_node_names[-1]=r.full_node_names[0]
    elif fault=='wrong_name':r.full_node_names[-1]='/fake'
    elif fault=='duplicate_id':r.unique_ids[-1]=r.unique_ids[0]
    elif fault=='zero_id':r.unique_ids[-1]=0
    elif fault=='overflow_id':r.unique_ids[-1]=2**64
    elif fault=='bool_id':r.unique_ids[-1]=True
    elif fault=='length':r.unique_ids.pop()
    with pytest.raises(TimeoutError):run(obj,sensors,types)
    d=json.loads((tmp_path/'preplay_consumers.json').read_text())
    assert d['status']=='FAIL' and not d['component_container']['matches']


def test_partial_component_load_waits_for_complete_response_without_data(tmp_path):
    obj,graph,required,sensors,types,clock=monitor(tmp_path)
    obj.component_responses.insert(0,SimpleNamespace(full_node_names=['/map_server'],unique_ids=[7]))
    host_clock = {'ns': 0}
    obj.preplay_consumers_ready.__func__.__globals__['time'].monotonic_ns = lambda: host_clock['ns']
    def spin(predicate,*args):
        assert not predicate() and len(obj.component_requests) == 1
        host_clock['ns'] = 99999999
        assert not predicate() and len(obj.component_requests) == 1
        host_clock['ns'] = 100000000
        assert predicate() and len(obj.component_requests) == 2
    obj.spin_until=spin
    run(obj,sensors,types)
    d=json.loads((tmp_path/'preplay_consumers.json').read_text())
    assert d['component_container']['request_count']==2 and d['status']=='PASS'
    assert d['component_container']['minimum_observed_request_interval_ns'] == 100000000
    assert clock.value==1000 and len(obj.component_requests)==2


@pytest.mark.parametrize('node', ['navigo_container','smoother_server'])
def test_correct_service_owner_and_type_required_without_lifecycle_rpc(tmp_path,node):
    obj,graph,required,sensors,types,clock=monitor(tmp_path)
    obj.service_graph[node]=[(obj.service_graph[node][0][0],['fake/srv/Wrong'])]
    with pytest.raises(TimeoutError):run(obj,sensors,types)
    d=json.loads((tmp_path/'preplay_consumers.json').read_text())
    assert not d['component_container' if node=='navigo_container' else 'standalone_smoother_service']['graph_type_match']
    assert d['standalone_smoother_service']['state_requested'] is False


def test_component_rpc_exception_preserved_as_failure(tmp_path):
    obj,graph,required,sensors,types,clock=monitor(tmp_path)
    def create(kind,service):
        def result():raise RuntimeError('actual ListNodes RPC failed')
        return SimpleNamespace(service_is_ready=lambda:True,
            call_async=lambda request:SimpleNamespace(done=lambda:True,result=result))
    obj.create_client=create
    with pytest.raises(RuntimeError,match='actual ListNodes RPC failed'):run(obj,sensors,types)
    assert json.loads((tmp_path/'preplay_consumers.json').read_text())['status']=='FAIL'


def test_pending_listnodes_rpc_is_not_resent_or_waited_synchronously(tmp_path):
    obj,graph,required,sensors,types,clock=monitor(tmp_path)
    ready={'done':False};calls=[]
    def create(kind,service):
        def call(request):
            assert service=='/navigo_container/_container/list_nodes'
            calls.append(request)
            return SimpleNamespace(done=lambda:ready['done'],result=lambda:obj.component_responses[0])
        return SimpleNamespace(service_is_ready=lambda:True,call_async=call)
    obj.create_client=create
    def spin(predicate,*args):
        assert not predicate() and not predicate() and len(calls)==1
        ready['done']=True
        assert predicate() and len(calls)==1
    obj.spin_until=spin
    run(obj,sensors,types)


def test_arbitrary_positive_unique_ids_and_response_order_are_valid(tmp_path):
    obj,graph,required,sensors,types,clock=monitor(tmp_path)
    response=obj.component_responses[0]
    response.full_node_names.reverse();response.unique_ids=[1000+i for i in range(8)]
    run(obj,sensors,types)
    d=json.loads((tmp_path/'preplay_consumers.json').read_text())
    assert d['component_container']['matches'] and d['component_container']['unique_ids']==response.unique_ids


def test_smoother_service_discovery_is_required_but_never_getstate_rpc(tmp_path):
    obj,graph,required,sensors,types,clock=monitor(tmp_path)
    original=obj.create_client
    def create(kind,service):
        client=original(kind,service)
        if service=='/smoother_server/get_state':client.service_is_ready=lambda:False
        return client
    obj.create_client=create
    with pytest.raises(TimeoutError):run(obj,sensors,types)
    d=json.loads((tmp_path/'preplay_consumers.json').read_text())
    assert not d['standalone_smoother_service']['service_ready']
    assert d['standalone_smoother_service']['state_requested'] is False


def test_runtime_manifest_records_actual_cleanup_flag():
    tree=ast.parse((HERE/'common.py').read_text())
    fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='runtime_manifest')
    expressions=[keyword.value for node in ast.walk(fn) if isinstance(node,ast.Call)
                 for keyword in node.keywords if keyword.arg=='cleanup_diagnostics']
    assert len(expressions)==1
    code=compile(ast.Expression(body=expressions[0]),'<actual manifest flag>','eval')
    for enabled in (False,True):
        assert eval(code,{'args':SimpleNamespace(cleanup_diagnostics=enabled)}) is enabled


def test_production_routing_keeps_old_recorder_resume_and_postreadiness():
    tree = ast.parse((HERE / 'replay.py').read_text())
    names = {}
    for n in ast.walk(tree):
        if isinstance(n, ast.Call):
            name = n.func.attr if isinstance(n.func, ast.Attribute) else getattr(n.func, 'id', '')
            names.setdefault(name, []).append(n.lineno)
    order = [min(names[name]) for name in ('recorder_ready', 'start_stack', 'preplay_consumers_ready', 'resume_source', 'readiness')]
    assert order == sorted(order)
    assert len(names['resume_source']) == 1
    text = (HERE / 'observer.py').read_text()
    body = text[text.index('    def preplay_consumers_ready'):text.index('    def until_source_time')]
    assert 'sleep(' not in body and 'sources_ready(' not in body
