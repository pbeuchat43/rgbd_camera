#!/usr/bin/env python3
"""Bounded hardware rate measurement. Requires a reserved host and explicit camera scope."""
# Copyright (c) 2026 Paul N. Beuchat. MIT License.
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from composition_interfaces.srv import LoadNode
from ament_index_python.packages import get_package_prefix
from rgbd_camera_interfaces.msg import Status as CameraStatus
from rgbd_object_detector_interfaces.msg import Status as DetectorStatus

parser=argparse.ArgumentParser()
parser.add_argument('--output',type=Path,required=True,help='New results directory; never overwritten')
parser.add_argument('--probe',type=Path,required=True,help='Explicitly built tools/rate_probe executable')
parser.add_argument('--device-id',required=True,help='Exact camera identity in the selected backend')
parser.add_argument('--engine',type=Path,help='Existing detector engine; enables the cone detector')
parser.add_argument('--backend',choices=['oak','realsense'],required=True)
parser.add_argument('--fps',type=float,default=10.0)
parser.add_argument('--seconds',type=float,default=30)
parser.add_argument('--tolerance-ms',type=float,help='Explicit pairing limit; default OAK 5, D455 16.6 (8.3 above 30 Hz)')
parser.add_argument('--layout',choices=['standalone','composed'],default='standalone')
parser.add_argument('--imu',action='store_true')
parser.add_argument('--images',action='store_true')
args=parser.parse_args()
if args.tolerance_ms is None:args.tolerance_ms=(8.3 if args.fps>30 else 16.6) if args.backend=='realsense' else 5.0
if not 5<=args.seconds<=3600:parser.error('--seconds must be within [5, 3600]')
if not args.probe.is_file():parser.error('--probe must name an existing executable')
if args.engine and not args.engine.is_file():parser.error('--engine must name an existing engine')
directory=args.output.resolve()
directory.mkdir()
processes=[]
def launch(name, command):
    log=(directory/(name+'.log')).open('w')
    p=subprocess.Popen([str(v) for v in command], stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    processes.append((name,p,log))
    return p
def ros(name,parameters=(),remaps=()):
    result=['--ros-args','-r','__ns:=/sweep','-r','__node:='+name]
    for k,v in parameters:
        result += ['-p',f'{k}:='+('true' if v is True else 'false' if v is False else "'"+v+"'" if isinstance(v,str) else str(v))]
    for k,v in remaps: result+=['-r',k+':='+v]
    return result
class Controller(Node):
    def __init__(self):
        super().__init__('rate_controller')
        self.camera=self.detector=None
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.a=self.create_subscription(CameraStatus,'/sweep/camera/status',lambda m:setattr(self,'camera',m),qos)
        self.b=self.create_subscription(DetectorStatus,'/sweep/detector/status',lambda m:setattr(self,'detector',m),qos)
    def wait(self,predicate,timeout):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.05)
            for name,p,_ in processes:
                if p.poll() is not None: raise RuntimeError(f'{name} exited {p.returncode}')
            if self.camera and self.camera.state=='error': raise RuntimeError(self.camera.detail)
            if self.detector and self.detector.state=='error': raise RuntimeError(self.detector.detail)
            if predicate(): return
        raise TimeoutError('readiness timed out')
    def load(self,package,plugin,name,parameters,remaps=()):
        client=self.create_client(LoadNode,'/sweep/container/_container/load_node')
        self.wait(client.service_is_ready,10)
        request=LoadNode.Request()
        request.package_name=package;request.plugin_name=plugin
        request.node_name=name;request.node_namespace='/sweep'
        request.parameters=[Parameter(k,value=v).to_parameter_msg() for k,v in parameters]
        request.extra_arguments=[Parameter('use_intra_process_comms',value=True).to_parameter_msg()]
        request.remap_rules=[k+':='+v for k,v in remaps]
        future=client.call_async(request);self.wait(future.done,40)
        response=future.result(); self.destroy_client(client)
        if not response.success: raise RuntimeError(response.error_message)
def sample(pid):
    text=Path(f'/proc/{pid}/stat').read_text().split(')',1)[1].split()
    cpu=int(text[11])+int(text[12]);rss=int(text[21])*os.sysconf('SC_PAGE_SIZE')
    children=subprocess.run(['ps','-o','pid=','--ppid',str(pid)],text=True,capture_output=True).stdout.split()
    for child in children:
        try:
            c,r=sample(int(child));cpu+=c;rss+=r
        except FileNotFoundError: pass
    return cpu,rss
def pct(values):
    if not values:return None
    s=sorted(values)
    return dict(p50=statistics.median(s),p95=s[int((len(s)-1)*.95)],p99=s[int((len(s)-1)*.99)],max=s[-1])
def read(name):
    path=directory/name
    if not path.exists():return []
    with path.open() as f:return list(csv.DictReader(f))
def standard_delivery(name, references, stamp_key, acquisition_key):
    """Match standard headers within this session to received authoritative samples."""
    path=directory/'native'/f'{name}.csv'
    if not path.is_file():raise RuntimeError(f'probe did not record enabled output: {name}')
    expected={int(r[stamp_key]):r for r in references}
    rows=[r for r in read(f'native/{name}.csv') if int(r['stamp_ns']) in expected]
    stamps=[int(r['stamp_ns']) for r in rows]
    arrivals=[int(r['arrival_ns']) for r in rows]
    result=dict(expected_from_received_samples=len(expected),matched_samples=len(set(stamps)),
        messages=len(rows),missing_from_received_samples=len(expected)-len(set(stamps)),
        duplicate_messages=len(stamps)-len(set(stamps)),
        reference_duplicates=len(references)-len(expected),
        arrival_rate_hz=(len(rows)-1)*1e9/(arrivals[-1]-arrivals[0]) if len(rows)>1 else 0,
        source_age_ms=pct([(int(r['arrival_ns'])-int(expected[int(r['stamp_ns'])][acquisition_key]))/1e6 for r in rows]),
        arrival_interval_ms=pct([(y-x)/1e6 for x,y in zip(arrivals,arrivals[1:])]))
    if name in ['rgb','depth','rgb_info','depth_info']:
        stream=name.split('_')[0]
        result['geometry_or_payload_mismatches']=sum(
            any(int(r[k])!=int(expected[int(r['stamp_ns'])][k]) for k in ['width','height']) or
            (not name.endswith('_info') and
             (int(r['bytes'])!=int(expected[int(r['stamp_ns'])][stream+'_bytes']) or
              r['encoding']!=('bgr8' if stream=='rgb' else '16UC1') or
              int(r['step'])!=int(r['width'])*(3 if stream=='rgb' else 2))) for r in rows)
    return result

def summarize(metadata):
    start,end=metadata['start_ns'],metadata['end_ns']
    statuses=[r for r in read('native/camera.csv') if start<=int(r['arrival_ns'])<=end]
    a,b=statuses[0],statuses[-1]
    low,high=int(a['last_acquisition_ns']),int(b['last_acquisition_ns'])
    admitted=int(b['frames'])-int(a['frames'])
    result=dict(**metadata,camera_admitted=admitted,
        camera_admission_hz=admitted*1e9/(int(b['observed_ns'])-int(a['observed_ns'])),
        camera_rejected=int(b['rejected'])-int(a['rejected']),
        native_sequence_gaps=int(b['gaps'])-int(a['gaps']),measurement_acquisition_bounds=[low,high],readers={})
    for name,file in [('native','native/frames.csv'),('detector','native/batches.csv')]:
        allrows=read(file)
        rows=[r for r in allrows if low<int(r['acquisition_ns'])<=high]
        if not rows:continue
        sequences=[int(r['sequence']) for r in rows]
        arrivals=[int(r['arrival_ns']) for r in rows]
        rate=(len(rows)-1)*1e9/(arrivals[-1]-arrivals[0]) if len(rows)>1 else 0
        data=dict(count=len(rows),missing_from_camera=admitted-len(set(sequences)),arrival_rate_hz=rate,
            source_age_ms=pct([(int(r['arrival_ns'])-int(r['acquisition_ns']))/1e6 for r in rows]),
            arrival_interval_ms=pct([(y-x)/1e6 for x,y in zip(arrivals,arrivals[1:])]))
        if 'callback_ns' in rows[0]:data['callback_ms']=pct([int(r['callback_ns'])/1e6 for r in rows])
        if name=='detector':
            data.update(valid_xyz=sum(int(r['valid']) for r in rows),invalid_xyz=sum(int(r['invalid']) for r in rows),
                empty_batches=sum(int(r['valid'])+int(r['invalid'])==0 for r in rows),
                completed_source_age_ms=pct([int(r['source_age_ns'])/1e6 for r in rows]),
                inference_ms=pct([int(r['inference_ns'])/1e6 for r in rows]))
        result['readers'][name]=data
    ds=[r for r in read('native/detector.csv') if start<=int(r['arrival_ns'])<=end]
    if ds:
        result['detector_counters']={k:int(ds[-1][k])-int(ds[0][k]) for k in ['received','processed','rejected','replaced','stale','superseded','failures']}
    if args.images:
        references=[r for r in read('native/frames.csv') if low<int(r['acquisition_ns'])<=high]
        result['standard_images']={}
        for stream in ['rgb','depth']:
            for suffix in ['', '_info']:
                result['standard_images'][stream+suffix]=standard_delivery(stream+suffix,references,
                    stream+'_stamp_ns','acquisition_ns' if stream=='rgb' else 'depth_acquisition_ns')
        result['standard_image_reference_complete']=len({r['sequence'] for r in references})==admitted
    if args.imu:
        result['imu']={}
        result['imu_unavailable_statuses']=sum(r['imu_state']!='streaming' for r in statuses)
        result['imu_native_sequence_gaps']=int(b['imu_sequence_gaps'])-int(a['imu_sequence_gaps'])
        for sensor in ['accel','gyro']:
            first,last=int(a['last_'+sensor+'_ns']),int(b['last_'+sensor+'_ns'])
            expected=int(b[sensor+'_samples'])-int(a[sensor+'_samples'])
            rows=[r for r in read(f'native/{sensor}.csv') if first<int(r['acquisition_ns'])<=last]
            arrivals=[int(r['arrival_ns']) for r in rows]
            unique=len({r['sequence'] for r in rows})
            result['imu'][sensor]=dict(admitted=expected,count=len(rows),missing_from_camera=expected-unique,
                duplicate_messages=len(rows)-unique,measurement_acquisition_bounds=[first,last],
                arrival_rate_hz=(len(rows)-1)*1e9/(arrivals[-1]-arrivals[0]) if len(rows)>1 else 0,
                source_age_ms=pct([(int(r['arrival_ns'])-int(r['acquisition_ns']))/1e6 for r in rows]),
                arrival_interval_ms=pct([(y-x)/1e6 for x,y in zip(arrivals,arrivals[1:])]),
                future_timestamp_violations=sum(int(r['acquisition_ns'])>int(r['received_ns'])+int(r['future_tolerance_ns']) for r in rows),
                standard=standard_delivery(sensor+'_standard',rows,'stamp_ns','acquisition_ns'))
    (directory/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(output=str(directory),admitted=admitted,camera_hz=result['camera_admission_hz'],rejected=result['camera_rejected'],
        readers={k:dict(count=v['count'],missing=v['missing_from_camera'],hz=v['arrival_rate_hz'],age_p95_ms=v['source_age_ms']['p95'],
            valid=v.get('valid_xyz'),empty=v.get('empty_batches')) for k,v in result['readers'].items()},
        detector=result.get('detector_counters'))),flush=True)

rclpy.init(); node=Controller()
metadata=dict(options={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
    environment={k:os.environ.get(k) for k in ['ROS_DOMAIN_ID','RMW_IMPLEMENTATION','FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE']})
try:
    launch('native-reader',[args.probe,'--ros-args','-p','output:='+str(directory/'native'),
        '-p','images:='+str(args.images).lower(),'-p','imu:='+str(args.imu).lower()])
    camera_parameters=[('backend',args.backend),('device_id',args.device_id),
        ('fps',args.fps),('pairing_tolerance_ms',args.tolerance_ms),('enable_imu',args.imu),('publish_images',args.images)]
    detector_parameters=[('engine_path',str(args.engine.resolve()))] if args.engine else []
    if args.layout=='composed':
        launch('container',[Path(get_package_prefix('rclcpp_components'))/'lib/rclcpp_components/component_container_mt']+ros('container'))
        node.load('rgbd_camera','rgbd_camera::CameraNode','camera',camera_parameters)
        if args.engine:node.load('rgbd_object_detector','rgbd_object_detector::DetectorNode','detector',detector_parameters,[('frames','camera/frames')])
    else:
        launch('camera',[Path(get_package_prefix('rgbd_camera'))/'lib/rgbd_camera/camera_node']+ros('camera',camera_parameters))
        if args.engine:launch('detector',[Path(get_package_prefix('rgbd_object_detector'))/'lib/rgbd_object_detector/detector_node']+ros('detector',detector_parameters,[('frames','camera/frames')]))
    node.wait(lambda:node.camera is not None and node.camera.state=='streaming' and
        (not args.imu or node.camera.imu_state=='streaming') and
        (not args.engine or node.detector is not None and node.detector.processed>=10),60)
    warmup=time.monotonic()+3
    node.wait(lambda:time.monotonic()>=warmup,5)
    metadata['start_ns']=time.monotonic_ns(); metadata['resources']=[]
    print('MEASURING '+str(directory),flush=True)
    next_sample=0
    def duration_done():
        global next_sample
        t=time.monotonic()
        if t>=next_sample:
            metadata['resources'].append(dict(time_ns=time.monotonic_ns(),processes={name:sample(p.pid) for name,p,_ in processes}))
            next_sample=t+2
        return time.monotonic_ns()-metadata['start_ns']>=args.seconds*1e9
    node.wait(duration_done,args.seconds+5)
    metadata['end_ns']=time.monotonic_ns()
    until=time.monotonic()+1;node.wait(lambda:time.monotonic()>=until,3)
except BaseException as error:
    metadata['error']=str(error)
    raise
finally:
    cleanup_errors=[]
    for name,p,log in reversed(processes):
        if p.poll() is None:
            try:
                os.killpg(p.pid,signal.SIGINT)
                p.wait(timeout=20)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                cleanup_errors.append(name+' required forced cleanup')
                try:
                    os.killpg(p.pid,signal.SIGKILL)
                    p.wait(timeout=5)
                except ProcessLookupError:
                    pass
                except subprocess.TimeoutExpired:
                    cleanup_errors.append(name+' did not exit after SIGKILL')
        log.close()
    if cleanup_errors:metadata['cleanup_errors']=cleanup_errors
    (directory/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
if metadata.get('cleanup_errors'):raise RuntimeError('; '.join(metadata['cleanup_errors']))
if 'error' not in metadata:summarize(metadata)
