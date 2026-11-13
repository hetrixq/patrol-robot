"""One optional RTSP client. OpenCV QR only, NO YOLO / PyTorch / NCNN.

Thread never touches ROS: it posts small events/JPEG bytes into a bounded queue.
The camera is closed while IDLE/PATROL unless a user enables preview; it opens
on CHECK and shows QR corners and decode result in an annotated Image panel.
"""
import os
os.environ.setdefault('OPENCV_FFMPEG_CAPTURE_OPTIONS',
                      'rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|max_delay;0')
os.environ.setdefault('OMP_NUM_THREADS', '1')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import queue
import threading
import time


class VisionWorker:
    def __init__(self, events: queue.Queue, url='rtsp://localhost:8554/cam',
                 qr_fps=2.0, max_width=960, view_width=720, jpeg_quality=60):
        self.events = events
        self.url = url
        self.qr_fps = max(0.5, min(float(qr_fps), 5.0))
        self.max_width = max(320, int(max_width))
        self.view_width = max(320, int(view_width))
        self.jpeg_quality = max(30, min(int(jpeg_quality), 90))
        self._mode = 'off'  # off, preview, qr
        self._lock = threading.Lock()
        self._quit = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name='qr-camera')
        self._thread.start()

    def mode(self):
        with self._lock:
            return self._mode

    def set_mode(self, mode):
        if mode not in ('off', 'preview', 'qr'):
            raise ValueError('unknown camera mode: ' + str(mode))
        with self._lock:
            self._mode = mode

    def close(self):
        self.set_mode('off')
        self._quit.set()
        self._thread.join(timeout=5)

    def _post(self, event, data=None):
        try:
            self.events.put_nowait((event, data))
        except queue.Full:
            # Skipping video frames is preferable to blocking Nav2 callbacks.
            if event == 'image':
                return
            try:
                self.events.get_nowait()
                self.events.put_nowait((event, data))
            except queue.Empty:
                pass

    def _run(self):
        cap = None
        cv2 = None
        detector = None
        last_mode = None
        next_processed_at = 0.0
        camera_failure_since = None
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
                    last_mode = mode
                    next_processed_at = 0.0
                if cv2 is None:
                    import cv2 as cv2_module
                    cv2 = cv2_module
                    cv2.setNumThreads(1)
                    detector = cv2.QRCodeDetector()
                if cap is None:
                    try:
                        params = [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 4000,
                                  cv2.CAP_PROP_READ_TIMEOUT_MSEC, 4000]
                        try:
                            cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG, params)
                        except (TypeError, cv2.error):
                            cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
                        if not cap.isOpened():
                            cap.release()
                            cap = None
                            raise RuntimeError('RTSP not opened')
                        self._post('camera_connected')
                        camera_failure_since = None
                    except Exception as exc:
                        if camera_failure_since is None:
                            camera_failure_since = time.monotonic()
                        if time.monotonic() - camera_failure_since > 7:
                            self._post('camera_error', str(exc))
                            camera_failure_since = time.monotonic()
                        self._quit.wait(0.5)
                        continue
                # Grab every RTSP frame to stay current; only retrieve/convert
                # a full image at QR_FPS (saves CPU versus read() each frame).
                ok = cap.grab()
                if not ok:
                    cap.release()
                    cap = None
                    if camera_failure_since is None:
                        camera_failure_since = time.monotonic()
                    if time.monotonic() - camera_failure_since > 7:
                        self._post('camera_error', 'RTSP read failed')
                        camera_failure_since = time.monotonic()
                    self._quit.wait(0.3)
                    continue
                camera_failure_since = None
                now = time.monotonic()
                if now < next_processed_at:
                    continue
                next_processed_at = now + 1.0 / self.qr_fps
                ok, frame = cap.retrieve()
                if not ok:
                    self._post('camera_error', 'RTSP frame retrieve failed')
                    cap.release()
                    cap = None
                    continue
                # Decode up to 960px wide. QR code must cover enough pixels.
                h, w = frame.shape[:2]
                if w > self.max_width:
                    frame = cv2.resize(frame, (self.max_width, round(h*self.max_width/w)))
                try:
                    code, corners, _ = detector.detectAndDecode(frame)
                    found = corners is not None and len(corners) > 0
                    code = (code or '').strip()
                    annotated = frame.copy()
                    if found:
                        points = corners.reshape((-1, 2)).astype('int32')
                        # Green: decoded; amber: found shape, cannot decode yet.
                        color = (0, 210, 0) if code else (0, 185, 255)
                        cv2.polylines(annotated, [points], True, color, 3)
                    if mode == 'qr':
                        label = 'CHECK: SHOW QR'
                    else:
                        label = 'PREVIEW: QR SCANNER'
                    if code:
                        label = ('QR: ' + code)[:55]
                    elif found:
                        label = 'QR DETECTED - MOVE CLOSER'
                    cv2.rectangle(annotated, (0, 0), (min(annotated.shape[1], 620), 40),
                                  (15, 15, 15), -1)
                    cv2.putText(annotated, label, (10, 27), cv2.FONT_HERSHEY_SIMPLEX,
                                0.67, (255, 255, 255), 2, cv2.LINE_AA)
                    hh, ww = annotated.shape[:2]
                    if ww > self.view_width:
                        annotated = cv2.resize(annotated,
                                              (self.view_width, round(hh*self.view_width/ww)))
                    encoded, data = cv2.imencode('.jpg', annotated,
                                                 [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                    # Image first: a successful QR must remain visible in Lichtblick
                    # even when the controller immediately closes the camera.
                    if encoded:
                        self._post('image', data.tobytes())
                    self._post('qr_frame', (bool(found), code, mode))
                except Exception as exc:
                    self._post('camera_error', 'QR/OpenCV: ' + repr(exc))
                    self.set_mode('off')
        finally:
            if cap is not None:
                cap.release()
