"""One RTSP reader, YOLO while patrolling, QR while stopped.

No frame decoding, model import, or inference while IDLE/STOPPED/DENIED.
All heavy work happens off the ROS executor. This is a demonstration feature,
not a collision avoidance or certified person-safety sensor.
"""
import os
os.environ.setdefault('OPENCV_FFMPEG_CAPTURE_OPTIONS',
    'rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|max_delay;0')
os.environ.setdefault('OMP_NUM_THREADS', '2')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import queue
import threading
import time


class VisionWorker:
    def __init__(self, events: queue.Queue, url='rtsp://localhost:8554/cam',
                 model_path='/root/weights/yolo11n_ncnn_model', fps=1.5,
                 qr_fps=3.0, imgsz=320, conf=0.45):
        self.events = events
        self.url = url
        self.model_path = model_path
        self.fps = max(0.2, float(fps))
        self.qr_fps = max(0.2, float(qr_fps))
        self.imgsz = int(imgsz)
        self.conf = float(conf)
        self._lock = threading.Lock()
        self._mode = 'off'  # off, person, hold, qr
        self._quit = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name='patrol-camera')
        self._thread.start()

    def mode(self):
        with self._lock:
            return self._mode

    def set_mode(self, mode):
        if mode not in ('off', 'person', 'hold', 'qr'):
            raise ValueError(mode)
        with self._lock:
            self._mode = mode

    def close(self):
        self.set_mode('off')
        self._quit.set()
        self._thread.join(timeout=4.0)

    def _post(self, event, data=None):
        try:
            self.events.put_nowait((event, data))
        except queue.Full:
            pass

    def _run(self):
        cap = None
        model = None
        qr_reader = None
        cv2 = None
        next_process = 0.0
        camera_failure_since = None
        last_mode = 'off'
        try:
            while not self._quit.is_set():
                mode = self.mode()
                if mode == 'off':
                    if cap is not None:
                        cap.release()
                        cap = None
                    camera_failure_since = None
                    last_mode = mode
                    self._quit.wait(0.15)
                    continue
                if mode != last_mode:
                    next_process = 0.0
                    last_mode = mode
                if cv2 is None:
                    import cv2 as cv2_module
                    cv2 = cv2_module
                    cv2.setNumThreads(1)
                if cap is None:
                    try:
                        params = [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 4000,
                                  cv2.CAP_PROP_READ_TIMEOUT_MSEC, 4000]
                        try:
                            cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG, params)
                        except (TypeError, cv2.error):
                            # Older OpenCV builds don't accept FFMPEG open-timeout options.
                            cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
                        if not cap.isOpened():
                            cap.release()
                            cap = None
                            raise RuntimeError('RTSP stream not opened')
                        camera_failure_since = None
                        self._post('camera_connected')
                    except Exception as exc:
                        if camera_failure_since is None:
                            camera_failure_since = time.monotonic()
                        if time.monotonic() - camera_failure_since >= 6.0:
                            self._post('camera_error', str(exc))
                            camera_failure_since = time.monotonic()
                        self._quit.wait(0.7)
                        continue
                ok, frame = cap.read()
                if not ok:
                    cap.release()
                    cap = None
                    if camera_failure_since is None:
                        camera_failure_since = time.monotonic()
                    if time.monotonic() - camera_failure_since >= 6.0:
                        self._post('camera_error', 'RTSP frame read failed')
                        camera_failure_since = time.monotonic()
                    self._quit.wait(0.2)
                    continue
                camera_failure_since = None
                now = time.monotonic()
                if mode == 'hold' or now < next_process:
                    continue
                if mode == 'person':
                    next_process = now + 1.0 / self.fps
                    if model is None:
                        try:
                            import torch
                            torch.set_num_threads(2)
                            from ultralytics import YOLO
                            model = YOLO(self.model_path, task='detect')
                            self._post('model_ready')
                        except Exception as exc:
                            self._post('vision_error', 'YOLO load: ' + str(exc))
                            self.set_mode('off')
                            continue
                    try:
                        h, w = frame.shape[:2]
                        if w > 640:
                            frame = cv2.resize(frame, (640, round(h * 640 / w)))
                        result = model.predict(frame, imgsz=self.imgsz, conf=self.conf,
                                               classes=[0], verbose=False, device='cpu')
                        person_found = any(r.boxes is not None and len(r.boxes) > 0
                                           for r in result)
                        self._post('person', bool(person_found))
                    except Exception as exc:
                        self._post('vision_error', 'YOLO inference: ' + str(exc))
                        self.set_mode('off')
                elif mode == 'qr':
                    next_process = now + 1.0 / self.qr_fps
                    if qr_reader is None:
                        qr_reader = cv2.QRCodeDetector()
                    try:
                        # Keep the original image: QR codes can be small in view.
                        value, _, _ = qr_reader.detectAndDecode(frame)
                        self._post('qr_frame')
                        if value:
                            self._post('qr_code', value)
                    except Exception as exc:
                        self._post('vision_error', 'QR: ' + str(exc))
                        self.set_mode('off')
        finally:
            if cap is not None:
                cap.release()
