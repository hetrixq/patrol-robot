"""ROS-free simulation of action cancellation and QR state transitions."""
import importlib
import sys
import time
import types
import unittest
from pathlib import Path
from types import SimpleNamespace as N

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def module(name, **items):
    mod = types.ModuleType(name)
    for key, value in items.items(): setattr(mod, key, value)
    sys.modules[name] = mod
    return mod


class Future:
    def __init__(self):
        self.callback = None
        self.value = None
    def add_done_callback(self, callback): self.callback = callback
    def result(self): return self.value
    def resolve(self, value):
        self.value = value
        self.callback(self)


class Pub:
    def __init__(self): self.msgs=[]
    def publish(self, data): self.msgs.append(data)


class Clock:
    def now(self): return N(to_msg=lambda: None)


class BaseNode:
    def __init__(self, name): self.name = name
    def create_publisher(self, *args): return Pub()
    def create_subscription(self, *args): return None
    def create_timer(self, *args): return None
    def get_logger(self): return N(info=lambda *a:None, warn=lambda *a:None, error=lambda *a:None)
    def get_clock(self): return Clock()
    def destroy_node(self): pass


class Msg:
    def __init__(self): self.data=''; self.header=N(frame_id='map', stamp=None)


class Pose(Msg):
    def __init__(self):
        super().__init__()
        self.pose=N(position=N(x=0,y=0),orientation=N(w=0))


class Mark(Msg):
    DELETEALL=3; ADD=0; SPHERE=2; TEXT_VIEW_FACING=9
    def __init__(self):
        super().__init__()
        self.ns=''; self.id=0
        self.pose=N(position=N(x=0,y=0,z=0),orientation=N(w=0))
        self.color=N(a=0,r=0,g=0,b=0)
        self.scale=N(x=0,y=0,z=0)


class Camera:
    def __init__(self, *args, **kwargs): self._mode='off'
    def set_mode(self, mode): self._mode=mode
    def mode(self): return self._mode
    def close(self): pass


class Client:
    def __init__(self, *args): self.requests=[]
    def server_is_ready(self): return True
    def send_goal_async(self, goal):
        f = Future()
        self.requests.append((goal,f))
        return f


class Handle:
    accepted=True
    def __init__(self): self.result_future=Future(); self.cancel_future=Future()
    def get_result_async(self): return self.result_future
    def cancel_goal_async(self): return self.cancel_future


module('rclpy', init=lambda:None, shutdown=lambda:None, spin=lambda node:None)
module('rclpy.node', Node=BaseNode)
module('rclpy.action', ActionClient=Client)
module('rclpy.qos', QoSProfile=lambda **kw:kw,
       DurabilityPolicy=N(TRANSIENT_LOCAL=1), ReliabilityPolicy=N(RELIABLE=1))
module('action_msgs'); module('action_msgs.msg', GoalStatus=N(STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6))
module('geometry_msgs'); module('geometry_msgs.msg', PointStamped=Msg, PoseStamped=Pose)
module('nav2_msgs'); module('nav2_msgs.action', NavigateToPose=N(Goal=lambda: N(pose=None)))
module('std_msgs'); module('std_msgs.msg', String=Msg, Bool=Msg)
module('visualization_msgs'); module('visualization_msgs.msg', Marker=Mark, MarkerArray=lambda: N(markers=[]))
module('vision', VisionWorker=Camera)
patrol_node = importlib.import_module('patrol_node')


class TestTransitions(unittest.TestCase):
    def setUp(self):
        self.node=patrol_node.Patrol()
        self.node._play_sound=lambda name: None
        self.node._sound_alarm=lambda: None
        self.node.route.add(0,0)
        self.node.route.add(2,2)

    def tearDown(self): self.node.destroy_node()

    def prepare_patrol(self):
        n=self.node
        n._start()
        self.assertEqual(n.state,'STARTING')
        self.assertEqual(n.camera.mode(),'person')
        self.assertEqual(len(n.navigator.requests),0)
        n._on_person(False)
        self.assertEqual(n.state,'STARTING')
        n._on_person(False)
        self.assertEqual(n.state,'PATROL')
        self.assertEqual(len(n.navigator.requests),1)
        return n

    def test_idle_no_vision(self):
        self.assertEqual(self.node.camera.mode(),'off')
        self.assertEqual(self.node.state,'IDLE')

    def test_start_then_navigation(self):
        self.prepare_patrol()

    def test_detection_cancel_then_qr(self):
        n=self.prepare_patrol()
        handle=Handle()
        n.navigator.requests[0][1].resolve(handle)
        n._on_person(True)
        n._on_person(True)
        self.assertEqual(n.state,'STOPPING')
        self.assertEqual(n.camera.mode(),'hold')
        self.assertIsNotNone(n.cancel_future)
        handle.cancel_future.resolve(N(goals_canceling=['id']))
        self.assertEqual(n.state,'STOPPING')  # cancel response alone is NOT enough
        handle.result_future.resolve(N(status=5))
        self.assertEqual(n.state,'CHECKING')
        self.assertEqual(n.camera.mode(),'qr')
        self.assertIsNone(n.qr_deadline)
        n.vision_events.put(('qr_frame',None))
        n._tick()
        self.assertGreater(n.qr_deadline,time.monotonic())

    def test_cancel_goal_not_yet_accepted(self):
        n=self.prepare_patrol()
        n._request_stop('user_stop')
        self.assertEqual(n.state,'STOPPING')
        handle=Handle()
        n.navigator.requests[0][1].resolve(handle)
        self.assertIsNotNone(n.cancel_future)
        handle.cancel_future.resolve(N(goals_canceling=['id']))
        handle.result_future.resolve(N(status=5))
        self.assertEqual(n.state,'STOPPED')

    def test_qr_timeout_and_reset(self):
        n=self.node
        n._begin_check() # no active Nav2 goal => directly into checking
        n.vision_events.put(('qr_frame',None)); n._tick()
        self.assertEqual(n.state,'CHECKING')
        n.qr_deadline=time.monotonic()-1
        n._tick()
        self.assertEqual(n.state,'DENIED')
        self.assertTrue(n.locked)
        self.assertEqual(n.camera.mode(),'off')
        n._command(N(data='reset'))
        self.assertEqual(n.state,'IDLE')
        self.assertFalse(n.locked)

    def test_qr_grant_requires_clear_frames(self):
        n=self.node
        n._begin_check()
        n.vision_events.put(('qr_frame',None)); n._tick()
        n._on_qr_code('PERSON_001')
        self.assertEqual(n.state,'STARTING')
        n._on_person(True) # still standing in front: must NOT move
        self.assertEqual(n.state,'STARTING')
        n._on_person(False); n._on_person(False)
        self.assertEqual(n.state,'PATROL')
        self.assertEqual(n.stats['granted'],1)

if __name__=='__main__': unittest.main()
