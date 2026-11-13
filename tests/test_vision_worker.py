"""Exercise actual vision worker without a camera using bundled QR image frames."""
import queue
import time
import unittest
from pathlib import Path

try:
    import cv2
except ImportError:
    cv2 = None

import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import importlib.util
vision_path=Path(__file__).resolve().parents[1]/"vision.py"
spec=importlib.util.spec_from_file_location("real_vision_worker_v5",vision_path)
vision_module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(vision_module)
VisionWorker=vision_module.VisionWorker

@unittest.skipIf(cv2 is None, 'OpenCV missing')
class VisionWorkerTests(unittest.TestCase):
    def test_image_contains_qr_and_annotation(self):
        image=cv2.imread(str(Path(__file__).resolve().parents[1]/'tools/PERSON_001.png'))
        events=queue.Queue(maxsize=32)
        original=cv2.VideoCapture
        class FakeCapture:
            def __init__(self,*args,**kwargs):self.closed=False
            def isOpened(self):return True
            def grab(self):
                time.sleep(.04)
                return not self.closed
            def retrieve(self):return True,image.copy()
            def release(self):self.closed=True
        cv2.VideoCapture=FakeCapture
        worker=None
        try:
            worker=VisionWorker(events,url='fake://camera',qr_fps=3)
            worker.set_mode('preview')
            received=[]
            until=time.monotonic()+4
            while time.monotonic()<until:
                try: event=events.get(timeout=.3)
                except queue.Empty:continue
                received.append(event)
                if event[0]=='qr_frame' and event[1][1]=='PERSON_001' and any(k=='image' for k,_ in received):
                    break
            self.assertTrue(any(k=='image' for k,_ in received),received)
            self.assertTrue(any(k=='qr_frame' and val[0] and val[1]=='PERSON_001' for k,val in received),received)
            jpeg=next(val for k,val in received if k=='image')
            self.assertTrue(jpeg.startswith(b'\xff\xd8'))
            self.assertGreater(len(jpeg),1000)
        finally:
            if worker:worker.close()
            cv2.VideoCapture=original

if __name__=='__main__':unittest.main()
