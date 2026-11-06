"""Read-only sample/graph validation; message decoding and ROS are injected."""
import hashlib
import math
import readiness_contract as c
from imu_gate import ImuMonotonicGate
from imu_converter import imu_cdr_evidence

RAW_IMU='/front_lidar/imu';IMU='/livox/imu';GT='/odom'
RAW_CAMERA='/front_camera/image/compressed';CAMERA='/camera/front/image/compressed';INFO='/camera/front/camera_info'
FRESH=(RAW_IMU,IMU,GT,'/front_lidar',RAW_CAMERA,CAMERA,INFO,'/clock')
TOPICS=FRESH+('/tf_static','/tf')
BRIDGE=('ue_zenoh_bridge','/');ADAPTER=('matrix_sensor_contract_adapter','/');PROJECTOR=('livox_scan_projector','/')
GRAPH={topic:[BRIDGE] for topic in [RAW_IMU,GT,'/front_lidar',RAW_CAMERA]}
GRAPH.update({topic:[ADAPTER] for topic in [IMU,'/livox/lidar',CAMERA,INFO,'/clock']})
GRAPH.update({'/tf_static':[ADAPTER,PROJECTOR],'/tf':[]})
TYPES={RAW_IMU:'sensor_msgs/msg/Imu',IMU:'sensor_msgs/msg/Imu',GT:'nav_msgs/msg/Odometry','/front_lidar':'sensor_msgs/msg/PointCloud2',
    RAW_CAMERA:'sensor_msgs/msg/CompressedImage',CAMERA:'sensor_msgs/msg/CompressedImage',INFO:'sensor_msgs/msg/CameraInfo',
    '/clock':'rosgraph_msgs/msg/Clock','/tf_static':'tf2_msgs/msg/TFMessage','/tf':'tf2_msgs/msg/TFMessage','/livox/lidar':'livox_ros_driver2/msg/CustomMsg'}

def stamp(h):return int(h.stamp.sec)*1_000_000_000+int(h.stamp.nanosec)
def vector(v):return [v.x,v.y,v.z]
def quaternion(q):return [q.x,q.y,q.z,q.w]

def static_expected(config):
    camera=config['robot']['sensors']['front_camera']
    r,p,y=[math.radians(camera['rotation'][k]) for k in ('roll','pitch','yaw')]
    cr,sr,cp,sp,cy,sy=math.cos(r/2),math.sin(r/2),math.cos(p/2),math.sin(p/2),math.cos(y/2),math.sin(y/2)
    x,y,z,w=[sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy]
    a,b,cc,d=[-.5,.5,-.5,.5]
    optical=[w*a+x*d+y*cc-z*b,w*b-x*cc+y*d+z*a,w*cc+x*b-y*a+z*d,w*d-x*a-y*b-z*cc]
    return {('base_link','livox_frame'):dict(values=[0.,0.,.1,0.,0.,0.,1.],owner=PROJECTOR),
        ('base_link','imu_link'):dict(values=[0.,0.,.1,0.,0.,0.,1.],owner=ADAPTER),
        ('base_link','front_camera_optical'):dict(values=[camera['position'][k] for k in ('x','y','z')]+optical,owner=ADAPTER)}

class Validator:
    def __init__(self,sensor_config,convert_imu):
        self.convert_imu=convert_imu;self.sensor_config=sensor_config
        self.raw_gate=ImuMonotonicGate('map');self.derived_gate=ImuMonotonicGate('imu_link')
        self.latest={};self.received={t:0 for t in TOPICS};self.accepted={t:0 for t in TOPICS}
        self.stats={};self.stop=c.StableStop();self.previous_gt=None;self.errors=[]
        self.raw={};self.derived={};self.cameras={};self.images={};self.infos={};self.clocks={}
        self.paired=set();self.camera_paired=set();self.clock_matched=set();self.static={};self.static_rules=static_expected(sensor_config)
        self.graph_pinned=None;self.measure_start=None;self.measure_end=None
        self.core_stats={}
        self.discovery_clock_rejections=0
        self.discovery_imu_rejections={RAW_IMU:0,IMU:0}
        self.imu_whole_CDR_equal=0;self.imu_padding_only_differences=0
    def core(self,row):return self.measure_start is not None and self.measure_start<=row['received_steady_ns'] and (self.measure_end is None or row['received_steady_ns']<=self.measure_end)
    def sample(self,topic,raw,msg,wall,steady,evidence):
        self.received[topic]+=1
        if (not isinstance(evidence,dict) or set(evidence)!={'message_info','matched_publisher_count'}
            or type(evidence['matched_publisher_count']) is not int):raise c.Hold('actual callback publisher-count/message-info evidence absent')
        row=dict(topic=topic,cdr_sha256=hashlib.sha256(raw).hexdigest(),cdr_bytes=len(raw),publisher_gid=None,
            **evidence,
            received_unix_ns=wall,received_steady_ns=steady,accepted=False)
        if self.core(row) and evidence['matched_publisher_count']!=len(GRAPH[topic]):raise c.Hold('core callback matched publisher count differs')
        if topic=='/tf':raise c.Hold('dynamic TF message observed in read-only sensor scope')
        if topic=='/tf_static':
            if not msg.transforms:raise c.Hold('empty static TF payload')
            row['transforms']=[]
            for tf in msg.transforms:
                edge=(tf.header.frame_id,tf.child_frame_id);values=vector(tf.transform.translation)+quaternion(tf.transform.rotation)
                rule=self.static_rules.get(edge)
                if rule is None or not c.finite(values) or any(abs(a-b)>1e-9 for a,b in zip(values,rule['values'])):raise c.Hold('unexpected static TF edge/value')
                self.static[edge]=dict(callback_publisher_gid=None,values=values,expected_owner_node=rule['owner'])
                row['transforms'].append(dict(parent=edge[0],child=edge[1],original_header_ns=stamp(tf.header),values=values))
            row['accepted']=True;self.accepted[topic]+=1;return row
        header=(int(msg.clock.sec)*1_000_000_000+int(msg.clock.nanosec)) if topic=='/clock' else stamp(msg.header)
        row.update(stamp_ns=header,frame=None if topic=='/clock' else msg.header.frame_id)
        if topic in (RAW_IMU,IMU):
            event=(self.raw_gate if topic==RAW_IMU else self.derived_gate).inspect(raw,wall,steady)
            row['imu_gate']=event
            if not event['accepted']:
                # Reliable DDS startup history is not accepted telemetry. Keep
                # the full original rejection event after the unchanged gate
                # has validated CDR/frame/finite fields; never refresh a lease.
                if (self.measure_start is None and event['fatal'] is True
                    and event['reason']=='original_header_stale_or_future'
                    and type(event['header_ns']) is int and wall-event['header_ns']>300_000_000):
                    self.discovery_imu_rejections[topic]+=1
                    row.update(admission_rejected=True,rejection_reason='old_IMU_during_bounded_discovery')
                    return row
                if topic==IMU or event['fatal']:raise c.Hold('IMU input fault: '+event['reason'])
                return row  # Raw old/equal bytes are retained; never refresh freshness.
        if topic in self.latest and header<=self.latest[topic]['stamp_ns']:raise c.Hold('nonadvancing header/clock: '+topic)
        if not -50_000_000<=wall-header<=300_000_000:
            # Discovery is bounded by the unchanged3s caller. Reject old first
            # clock data without refreshing any accepted clock/freshness state.
            # The fixed10s core cannot begin until a genuinely fresh clock has
            # matched an accepted original IMU header. Future remains fatal.
            if topic=='/clock' and self.measure_start is None and wall-header>300_000_000:
                self.discovery_clock_rejections+=1
                row.update(admission_rejected=True,rejection_reason='old_clock_during_bounded_discovery',original_header_age_ns=wall-header)
                return row
            raise c.Hold('stale/future original header at callback: '+topic)
        if topic in (RAW_IMU,IMU):
            if topic==RAW_IMU:
                row['imu_expected_encoding']=imu_cdr_evidence(self.convert_imu(raw))
                item=dict(row,expected_cdr_sha256=row['imu_expected_encoding']['original_cdr_sha256'],
                    expected_padding_normalized_cdr_sha256=row['imu_expected_encoding']['padding_normalized_cdr_sha256'])
            else:
                row['imu_observed_encoding']=imu_cdr_evidence(raw)
                item=dict(row,padding_normalized_cdr_sha256=row['imu_observed_encoding']['padding_normalized_cdr_sha256'])
            (self.raw if topic==RAW_IMU else self.derived)[header]=item
            if header in self.raw and header in self.derived:
                if self.raw[header]['expected_padding_normalized_cdr_sha256']!=self.derived[header]['padding_normalized_cdr_sha256']:raise c.Hold('baseline raw->derived IMU wire fields differ outside3 CDR alignment bytes')
                equal=self.raw[header]['expected_cdr_sha256']==self.derived[header]['cdr_sha256']
                self.imu_whole_CDR_equal+=int(equal);self.imu_padding_only_differences+=int(not equal)
                self.paired.add(header)
        elif topic==GT:
            sample=dict(row,position=vector(msg.pose.pose.position),quaternion=quaternion(msg.pose.pose.orientation),
                linear=vector(msg.twist.twist.linear),angular=vector(msg.twist.twist.angular),child=msg.child_frame_id)
            roll,pitch,yaw=c.rpy(sample['quaternion'])
            if (sample['frame']!='map' or sample['child']!='BASE_LINK' or not c.finite(sample['position']+sample['linear']+sample['angular'])
                or sample['position'][2]<=.2 or abs(roll)>.25 or abs(pitch)>.25):raise c.Hold('GT frame/standing/attitude differs')
            if self.previous_gt:
                sample['position_speed_mps']=math.dist(sample['position'],self.previous_gt['position'])/((header-self.previous_gt['stamp_ns'])/1e9)
            self.previous_gt=sample;self.stop.update(sample);row.update(sample)
        elif topic=='/front_lidar':
            if msg.header.frame_id!='map' or msg.width<=0 or msg.height<=0 or len(msg.data)!=msg.row_step*msg.height:raise c.Hold('raw lidar frame/empty/layout differs')
        elif topic in (RAW_CAMERA,CAMERA):
            if row['frame']!=('map' if topic==RAW_CAMERA else 'front_camera_optical') or len(msg.data)<=100:raise c.Hold('compressed camera frame/empty payload')
            item=dict(row,data_sha256=hashlib.sha256(bytes(msg.data)).hexdigest(),format=msg.format)
            (self.cameras if topic==RAW_CAMERA else self.images)[header]=item
            if header in self.cameras and header in self.images:
                if any(self.cameras[header][key]!=self.images[header][key] for key in ('data_sha256','format')):raise c.Hold('camera passthrough header/data differs')
                self.camera_paired.add(header)
            row.update(data_sha256=item['data_sha256'],format=item['format'])
        elif topic==INFO:
            cam=self.sensor_config['robot']['sensors']['front_camera']
            expected_k=[cam['fx'],0.,cam['cx'],0.,cam['fy'],cam['cy'],0.,0.,1.]
            if row['frame']!='front_camera_optical' or msg.width!=cam['width'] or msg.height!=cam['height'] or list(msg.k)!=expected_k:raise c.Hold('camera info original header/frame/intrinsics differs')
            self.infos[header]=dict(row)
        elif topic=='/clock':self.clocks[header]=dict(row)
        row['accepted']=True;self.accepted[topic]+=1;self.latest[topic]=row
        if self.core(row):
            item=self.core_stats.setdefault(topic,dict(count=0,first_header_ns=header,last_header_ns=header))
            item['count']+=1;item['last_header_ns']=header
        stat=self.stats.setdefault(topic,dict(first_header_ns=header,last_header_ns=header,min_callback_age_ns=wall-header,max_callback_age_ns=wall-header,min_header_delta_ns=None,max_header_delta_ns=None,
            first_receipt_steady_ns=steady,last_receipt_steady_ns=steady,min_receipt_delta_ns=None,max_receipt_delta_ns=None))
        if self.accepted[topic]>1:
            delta=header-stat['last_header_ns'];stat['min_header_delta_ns']=delta if stat['min_header_delta_ns'] is None else min(stat['min_header_delta_ns'],delta)
            stat['max_header_delta_ns']=delta if stat['max_header_delta_ns'] is None else max(stat['max_header_delta_ns'],delta)
            receipt_delta=steady-stat['last_receipt_steady_ns']
            if receipt_delta<0:raise c.Hold('callback steady receipt regressed')
            stat['min_receipt_delta_ns']=receipt_delta if stat['min_receipt_delta_ns'] is None else min(stat['min_receipt_delta_ns'],receipt_delta)
            stat['max_receipt_delta_ns']=receipt_delta if stat['max_receipt_delta_ns'] is None else max(stat['max_receipt_delta_ns'],receipt_delta)
        stat['last_header_ns']=header;stat['last_receipt_steady_ns']=steady;stat['min_callback_age_ns']=min(stat['min_callback_age_ns'],wall-header);stat['max_callback_age_ns']=max(stat['max_callback_age_ns'],wall-header)
        return row
    def graph(self,snapshot,pin=False):
        if set(snapshot)!=set(GRAPH):raise c.Hold('graph topic inventory differs')
        for topic,owners in GRAPH.items():
            rows=snapshot[topic]
            if sorted((r['node_name'],r['node_namespace']) for r in rows)!=sorted(owners):raise c.Hold('publisher count/node differs: '+topic)
            if any(row['topic_type']!=TYPES[topic] for row in rows):raise c.Hold('graph publisher type differs: '+topic)
            if any(not row['gid'] for row in rows) or len({r['gid'] for r in rows})!=len(rows):raise c.Hold('missing/duplicate publisher GID')
        if self.graph_pinned is not None and snapshot!=self.graph_pinned:raise c.Hold('publisher graph/GID changed after discovery')
        if pin:self.graph_pinned=snapshot
    def check(self,wall,steady):
        if self.errors:raise c.Hold(self.errors[0])
        for topic in FRESH:
            row=self.latest.get(topic)
            if row is None or not -50_000_000<=wall-row['stamp_ns']<=300_000_000 or not 0<=steady-row['received_steady_ns']<=300_000_000:raise c.Hold('current accepted header/receipt stale: '+topic)
            stat=self.stats[topic];stat['max_checked_receipt_age_ns']=max(stat.get('max_checked_receipt_age_ns',0),steady-row['received_steady_ns'])
            stat['min_checked_header_age_ns']=min(stat.get('min_checked_header_age_ns',wall-row['stamp_ns']),wall-row['stamp_ns'])
            stat['max_checked_header_age_ns']=max(stat.get('max_checked_header_age_ns',wall-row['stamp_ns']),wall-row['stamp_ns'])
        if not self.stop.ready():raise c.Hold('strict measured GT stop not continuous >=.3s')
        camera_complete=self.camera_paired&self.infos.keys()
        if set(self.static)!=set(self.static_rules) or not self.paired or not camera_complete:raise c.Hold('static edges/initial converter or camera pairs absent')
        for header,item in self.clocks.items():
            if header in self.raw:self.clock_matched.add(header)
        if not self.clock_matched:raise c.Hold('clock has no accepted original IMU header')
        for collection,complete,label in [(self.raw,self.paired,'raw IMU without derived'),(self.derived,self.paired,'derived IMU without accepted raw'),
                (self.cameras,camera_complete,'raw camera without passthrough/info'),(self.images,camera_complete,'camera without raw/info'),
                (self.infos,camera_complete,'camera info without original image'),(self.clocks,self.clock_matched,'clock without accepted original IMU')]:
            for header,item in collection.items():
                if self.core(item) and header not in complete and steady-item['received_steady_ns']>300_000_000:raise c.Hold(label)
    def finish(self,wall,steady):
        missing=[]
        camera_complete=self.camera_paired&self.infos.keys()
        for collection,complete,label in [(self.raw,self.paired,'raw IMU'),(self.derived,self.paired,'derived IMU'),(self.cameras,camera_complete,'raw camera'),
                (self.images,camera_complete,'camera'),(self.infos,camera_complete,'camera info'),(self.clocks,self.clock_matched,'clock')]:
            for header,item in collection.items():
                if self.core(item) and header not in complete:missing.append([label,header])
        for topic in FRESH:
            item=self.core_stats.get(topic,{})
            if item.get('count',0)<2 or item.get('last_header_ns',0)-item.get('first_header_ns',0)<9_300_000_000:missing.append(['insufficient core sample/header span',topic])
        return dict(complete=not missing,unmatched_core_samples=missing,discovery_clock_rejections=self.discovery_clock_rejections,discovery_imu_rejections=self.discovery_imu_rejections,raw_gate=self.raw_gate.diagnostics(wall,steady),derived_gate=self.derived_gate.diagnostics(wall,steady),
            received=self.received,accepted=self.accepted,topic_statistics=self.stats,imu_semantic_bit_matches=len(self.paired),
            imu_whole_CDR_equal=self.imu_whole_CDR_equal,imu_padding_only_differences=self.imu_padding_only_differences,
            imu_excluded_wire_byte_offsets=[25,26,27],imu_all_other_wire_bits_compared_exactly=True,
            core_topic_statistics=self.core_stats,
            camera_original_payload_matches=len(self.camera_paired),clock_original_IMU_header_matches=len(self.clock_matched),
            measure_start_steady_ns=self.measure_start,measure_end_steady_ns=self.measure_end,
            static_edges=[dict(parent=k[0],child=k[1],**v) for k,v in sorted(self.static.items())],
            graph_pinned=self.graph_pinned,ready_for_motion=False)
