"""ROS-free unit tests; confirm Nav2 cancellation & manual CHECK flow."""
import importlib
import sys
import time
import types
import unittest
from pathlib import Path
from types import SimpleNamespace as N

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def module(name,**items):
    m=types.ModuleType(name)
    for k,v in items.items(): setattr(m,k,v)
    sys.modules[name]=m
    return m

class Future:
    def __init__(self): self.cb=None; self.value=None
    def add_done_callback(self,callback): self.cb=callback
    def result(self): return self.value
    def resolve(self,val): self.value=val; self.cb(self)

class Pub:
    def __init__(self): self.msgs=[]
    def publish(self,value): self.msgs.append(value)
    def last(self): return self.msgs[-1].data

class Clock:
    def now(self): return N(to_msg=lambda:None)

class BaseNode:
    def __init__(self,name): self.name=name
    def create_publisher(self,*args): return Pub()
    def create_subscription(self,*args): return None
    def create_timer(self,*args): return None
    def get_logger(self): return N(info=lambda *a:None,warn=lambda *a:None)
    def get_clock(self): return Clock()
    def destroy_node(self): pass

class Msg:
    def __init__(self):
        self.data=''
        self.header=N(frame_id='map',stamp=None)

class Pose(Msg):
    def __init__(self):
        super().__init__()
        self.pose=N(position=N(x=0,y=0),orientation=N(w=0))

class Marker(Msg):
    DELETEALL=3; ADD=0; SPHERE=2; TEXT_VIEW_FACING=9
    def __init__(self):
        super().__init__()
        self.ns='';self.id=0
        self.pose=N(position=N(x=0,y=0,z=0),orientation=N(w=0))
        self.color=N(a=0,r=0,g=0,b=0)
        self.scale=N(x=0,y=0,z=0)

class Camera:
    def __init__(self,*a,**kw): self._mode='off'
    def mode(self):return self._mode
    def set_mode(self,value): self._mode=value
    def close(self): pass

class Client:
    def __init__(self,*args): self.requests=[]
    def server_is_ready(self):return True
    def send_goal_async(self,goal):
        f=Future();self.requests.append((goal,f));return f

class Handle:
    accepted=True
    def __init__(self):self.result_future=Future();self.cancel_future=Future()
    def get_result_async(self):return self.result_future
    def cancel_goal_async(self):return self.cancel_future

module('rclpy',init=lambda:None,shutdown=lambda:None,spin=lambda node:None)
module('rclpy.node',Node=BaseNode)
module('rclpy.action',ActionClient=Client)
module('rclpy.qos',QoSProfile=lambda **k:k,DurabilityPolicy=N(TRANSIENT_LOCAL=1),ReliabilityPolicy=N(RELIABLE=1,BEST_EFFORT=2))
module('action_msgs');module('action_msgs.msg',GoalStatus=N(STATUS_SUCCEEDED=4,STATUS_CANCELED=5,STATUS_ABORTED=6))
module('geometry_msgs');module('geometry_msgs.msg',PointStamped=Msg,PoseStamped=Pose)
module('nav2_msgs');module('nav2_msgs.action',NavigateToPose=N(Goal=lambda:N(pose=None)))
module('sensor_msgs');module('sensor_msgs.msg',CompressedImage=Msg)
module('std_msgs');module('std_msgs.msg',String=Msg,Bool=Msg)
module('visualization_msgs');module('visualization_msgs.msg',Marker=Marker,MarkerArray=lambda:N(markers=[]))
module('vision',VisionWorker=Camera)
node_mod=importlib.import_module('patrol_node')

class ManualPatrolTests(unittest.TestCase):
    def setUp(self):
        self.n=node_mod.Patrol()
        self.n._play_sound=lambda name:None
        self.n._sound_alarm=lambda:None
        self.n.route.add(1,1)
        self.n.route.add(2,2)
    def tearDown(self):self.n.destroy_node()
    def cmd(self,command):self.n._command(N(data=command))
    def first_qr(self,code='',found=False,source='qr'):
        self.n._on_qr_frame(found,code,source)
    def start_nav(self):
        self.cmd('start')
        self.assertEqual(self.n.state,'PATROL')
        self.assertEqual(self.n.camera.mode(),'off')
        self.assertEqual(len(self.n.navigator.requests),1)
    def accepted_goal(self):
        h=Handle()
        self.n.navigator.requests[-1][1].resolve(h)
        return h
    def test_idle_no_camera_or_yolo(self):
        self.assertEqual(self.n.camera.mode(),'off')
        self.assertEqual(self.n.state,'IDLE')
    def test_preview_does_not_trigger_check(self):
        self.cmd('camera_on')
        self.assertEqual(self.n.camera.mode(),'preview')
        self.first_qr('PERSON_001',True,'preview')
        self.assertEqual(self.n.state,'IDLE')
        self.assertEqual(self.n.stats['granted'],0)
        self.cmd('camera_off')
        self.assertEqual(self.n.camera.mode(),'off')
    def test_dedicated_manual_topic(self):
        self.n._manual_check(N(data=False))
        self.assertEqual(self.n.state,'IDLE')
        self.n._manual_check(N(data=True))
        self.assertEqual(self.n.state,'CHECKING')

    def test_check_from_idle_without_waypoints(self):
        self.n.route.clear()
        self.cmd('check')
        self.assertEqual(self.n.state,'CHECKING')
        self.assertEqual(self.n.camera.mode(),'qr')
        self.first_qr('',False)
        self.assertIsNotNone(self.n.qr_deadline)
        self.first_qr('PERSON_001',True)
        self.assertEqual(self.n.state,'IDLE')
        self.assertEqual(self.n.stats['granted'],1)
        self.assertEqual(self.n.camera.mode(),'off')
    def test_check_while_moving_waits_for_nav_result(self):
        self.start_nav()
        handle=self.accepted_goal()
        self.cmd('check')
        self.assertEqual(self.n.state,'STOPPING')
        self.assertEqual(self.n.camera.mode(),'off')
        handle.cancel_future.resolve(N(goals_canceling=['goal']))
        self.assertEqual(self.n.state,'STOPPING')
        handle.result_future.resolve(N(status=5))
        self.assertEqual(self.n.state,'CHECKING')
        self.assertEqual(self.n.camera.mode(),'qr')
        self.first_qr('PERSON_001',True)
        self.assertEqual(self.n.state,'PATROL')
        self.assertEqual(len(self.n.navigator.requests),2)
        self.assertEqual(self.n.camera.mode(),'off')
    def test_check_before_goal_acceptance(self):
        self.start_nav()
        self.cmd('check')
        self.assertEqual(self.n.state,'STOPPING')
        h=self.accepted_goal()
        self.assertIsNotNone(self.n.cancel_future)
        h.cancel_future.resolve(N(goals_canceling=['goal']))
        h.result_future.resolve(N(status=5))
        self.assertEqual(self.n.state,'CHECKING')
    def test_wrong_qr_denies_until_reset(self):
        self.cmd('check')
        self.first_qr('PERSON_999',True)
        self.assertEqual(self.n.state,'DENIED')
        self.cmd('start')
        self.assertEqual(self.n.state,'DENIED')
        self.cmd('reset')
        self.assertEqual(self.n.state,'IDLE')
        self.assertFalse(self.n.locked)
    def test_timeout_15_seconds(self):
        self.cmd('check')
        self.first_qr('',False)
        self.n.qr_deadline=time.monotonic()-0.1
        self.n._tick()
        self.assertEqual(self.n.state,'DENIED')
        self.assertEqual(self.n.stats['denied'],1)
    def test_camera_failure_is_fault_not_denied(self):
        self.cmd('check')
        self.n.vision_events.put(('camera_error','failure'))
        self.n._tick()
        self.assertEqual(self.n.state,'SAFETY_FAULT')
        self.assertEqual(self.n.stats['denied'],0)
    def test_preview_frame_cannot_grant_check(self):
        self.cmd('camera_on')
        self.cmd('check')
        self.first_qr('PERSON_001',True,'preview')
        self.assertEqual(self.n.state,'CHECKING')
    def test_clear_and_random(self):
        self.cmd('random')
        self.assertEqual(self.n.route.mode,'random')
        self.cmd('undo')
        self.assertEqual(len(self.n.route.points),1)
        self.cmd('clear')
        self.assertEqual(self.n.route.points,[])
    def test_oversized_route_rejected(self):
        for _ in range(8):self.n.route.add(0,0)
        with self.assertRaises(ValueError):self.n.route.add(0,0)
    def test_stop_in_checking_cancels_without_denial(self):
        self.cmd('check')
        self.cmd('stop')
        self.assertEqual(self.n.state,'STOPPED')
        self.assertFalse(self.n.locked)
    def test_action_abort_on_cancel_triggers_fault(self):
        self.start_nav()
        h=self.accepted_goal()
        self.cmd('check')
        h.cancel_future.resolve(N(goals_canceling=[]))
        h.result_future.resolve(N(status=6))
        self.assertEqual(self.n.state,'SAFETY_FAULT')

if __name__=='__main__':unittest.main()
