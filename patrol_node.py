#!/usr/bin/env python3
"""Manual-triggered QR patrol for existing Raspberry Pi / RBM stack.

Uses current SLAM, Nav2, RTSP and Lichtblick. NO YOLO/torch/NCNN imports.
Safety: before accepting QR while moving, await Nav2 action termination/cancel.
This app is NOT a certified physical emergency stop or collision detector.
"""
import json
import os
import queue
import shutil
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker, MarkerArray

from core import Route, qr_decision
from vision import VisionWorker

HERE = Path(__file__).resolve().parent
QR_TIMEOUT = float(os.environ.get('PATROL_QR_TIMEOUT', '15'))
QR_CAMERA_TIMEOUT = float(os.environ.get('PATROL_QR_CAMERA_TIMEOUT', '10'))
NAV_CANCEL_TIMEOUT = float(os.environ.get('PATROL_NAV_CANCEL_TIMEOUT', '8'))
NAV_GOAL_TIMEOUT = float(os.environ.get('PATROL_NAV_GOAL_TIMEOUT', '120'))
ALLOWED_QR = os.environ.get('PATROL_ALLOWED_QR', 'PERSON_001')


class Patrol(Node):
    def __init__(self):
        super().__init__('patrol_classroom_v5')
        self.route = Route()
        self.state = 'IDLE'
        self.locked = False
        self.resume_after_check = False
        self.return_state = 'IDLE'
        self.stop_reason = None
        self.stop_since = None
        self.error_text = None
        self.qr_camera_deadline = None
        self.qr_deadline = None
        self.qr_last_frame_at = None
        self.qr_found = False
        self.qr_text = ''
        self.nav_sent_at = None
        self.goal_future = None
        self.goal_handle = None
        self.goal_result_future = None
        self.cancel_future = None
        self.last_sound_try = 0
        self.audio_process = None
        self.stats = {'granted': 0, 'denied': 0, 'goals_reached': 0, 'laps': 0, 'checks': 0}
        self.vision_events = queue.Queue(maxsize=12)
        self.camera = VisionWorker(
            self.vision_events,
            url=os.environ.get('PATROL_RTSP_URL', 'rtsp://localhost:8554/cam'),
            qr_fps=float(os.environ.get('PATROL_QR_FPS', '2')),
            max_width=int(os.environ.get('PATROL_QR_SCAN_WIDTH', '960')),
            view_width=int(os.environ.get('PATROL_CAMERA_VIEW_WIDTH', '720')),
            jpeg_quality=int(os.environ.get('PATROL_JPEG_QUALITY', '60')),
        )
        sticky = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/patrol/state', sticky)
        self.events_pub = self.create_publisher(String, '/patrol/events', 10)
        self.stats_pub = self.create_publisher(String, '/patrol/stats', sticky)
        self.markers_pub = self.create_publisher(MarkerArray, '/patrol/markers', sticky)
        self.qr_status_pub = self.create_publisher(String, '/patrol/qr/status', sticky)
        self.qr_value_pub = self.create_publisher(String, '/patrol/qr/value', sticky)
        self.qr_visible_pub = self.create_publisher(Bool, '/patrol/qr/visible', 10)
        # Best-effort video: dropping video frames must not stall navigation.
        video_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.image_pub = self.create_publisher(CompressedImage,
                                               '/patrol/camera/image/compressed', video_qos)
        self.create_subscription(PointStamped, '/clicked_point', self._clicked, 10)
        self.create_subscription(String, '/robot_commands', self._command, 10)
        # Dedicated synthetic trigger from Lichtblick CHECK button.
        self.create_subscription(Bool, '/patrol/manual_check', self._manual_check, 10)
        self.navigator = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.create_timer(0.10, self._tick)
        self.create_timer(1.0, self._heartbeat)
        self._publish_markers()
        self._qr_status('CAMERA OFF')
        self._pub(self.qr_value_pub, '')
        self._state('IDLE', 'Готов. Click Point: 2–10 точек. CHECK — ручная проверка QR.')

    def _pub(self, pub, text):
        msg = String()
        msg.data = str(text)
        pub.publish(msg)

    def _qr_status(self, text):
        self._pub(self.qr_status_pub, text)

    def _event(self, message):
        self._pub(self.events_pub, f'{time.strftime("%H:%M:%S")} {message}')
        self.get_logger().info(message)

    def _state(self, name, why):
        self.state = name
        if name in ('DENIED', 'NAV_ERROR', 'SAFETY_FAULT'):
            self.locked = True
        self._pub(self.state_pub, name)
        self._pub(self.stats_pub, json.dumps(self.stats, ensure_ascii=False))
        self._event(f'[{name}] {why}')

    def _heartbeat(self):
        self._pub(self.state_pub, self.state)
        self._pub(self.stats_pub, json.dumps(self.stats, ensure_ascii=False))
        if self.state == 'CHECKING' and self.qr_deadline is not None:
            remain = max(0, int(self.qr_deadline - time.monotonic() + 0.999))
            self._qr_status(f'CHECKING: {remain} sec | {"QR DETECTED" if self.qr_found else "NO QR"}')

    def _publish_markers(self):
        msg = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        msg.markers.append(clear)
        for i, (x, y) in enumerate(self.route.points):
            for kind in ('point', 'text'):
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'patrol_' + kind
                m.id = i
                m.action = Marker.ADD
                m.pose.position.x, m.pose.position.y = x, y
                m.pose.position.z = 0.12 if kind == 'point' else 0.43
                m.pose.orientation.w = 1.0
                m.color.a = 1.0
                if kind == 'point':
                    m.type = Marker.SPHERE
                    m.scale.x = m.scale.y = 0.23
                    m.scale.z = 0.08
                    m.color.r, m.color.g, m.color.b = .1, .5, 1.0
                else:
                    m.type = Marker.TEXT_VIEW_FACING
                    m.scale.z = .32
                    m.color.r = m.color.g = m.color.b = 1.0
                    m.text = str(i+1)
                msg.markers.append(m)
        self.markers_pub.publish(msg)

    def _clicked(self, msg):
        if self.state not in ('IDLE', 'STOPPED') or self.locked or self._has_nav():
            self._event('Изменить точки можно только в IDLE/STOPPED без активной цели')
            return
        if msg.header.frame_id.lstrip('/') != 'map':
            self._event('Click Point: выберите frame=map')
            return
        try:
            self.route.add(msg.point.x, msg.point.y)
        except ValueError as exc:
            self._event(str(exc))
            return
        self._publish_markers()
        self._event(f'Точка {len(self.route.points)}: {msg.point.x:.2f}, {msg.point.y:.2f}')

    def _command(self, msg):
        cmd = msg.data.strip().lower()
        if cmd in ('loop', 'random'):
            if self.state in ('IDLE', 'STOPPED'):
                self.route.set_mode(cmd)
                self._event('Маршрут: ' + cmd)
            return
        if cmd in ('clear', 'undo'):
            if self.state not in ('IDLE', 'STOPPED') or self.locked or self._has_nav():
                self._event('Нельзя редактировать точки во время движения или проверки')
                return
            self.route.clear() if cmd == 'clear' else self.route.undo()
            self._publish_markers()
            return
        if cmd == 'start':
            self._start()
        elif cmd == 'check':
            self._begin_manual_check()
        elif cmd == 'camera_on':
            if self.state in ('IDLE', 'STOPPED') and not self.locked:
                self.camera.set_mode('preview')
                self._qr_status('PREVIEW: WAITING FOR CAMERA')
                self._event('Камера включена в режиме предварительного просмотра QR')
            else:
                self._event('CAMERA ON доступна в IDLE/STOPPED; CHECK работает при патруле')
        elif cmd == 'camera_off':
            if self.state in ('IDLE', 'STOPPED'):
                self.camera.set_mode('off')
                self._qr_status('CAMERA OFF')
            else:
                self._event('CAMERA OFF недоступна во время CHECKING/движения')
        elif cmd == 'stop':
            if self.state == 'DENIED':
                self._event('Сирена заблокирована до RESET')
            elif self.state != 'STOPPING':
                self._request_stop('user_stop')
        elif cmd == 'reset':
            self._request_stop('reset')
        else:
            self._event('Неизвестная команда: ' + cmd)

    def _manual_check(self, msg):
        if msg.data:
            self._begin_manual_check()

    def _start(self):
        if self.locked or self.state not in ('IDLE', 'STOPPED') or self._has_nav():
            self._event('START недоступен. Сначала RESET или дождитесь полной остановки')
            return
        if not self.route.ready():
            self._event('Для патруля требуется 2–10 точек')
            return
        if not self.navigator.server_is_ready():
            self._event('Nav2 /navigate_to_pose недоступен')
            return
        self.camera.set_mode('off')
        self._qr_status('CAMERA OFF: PATROL')
        self._state('PATROL', 'Маршрут запущен. YOLO отсутствует; CHECK нажимает оператор.')
        self._send_next_goal()

    def _begin_manual_check(self):
        if self.locked or self.state not in ('IDLE', 'STOPPED', 'PATROL'):
            self._event('CHECK сейчас недоступен (проверка/ошибка/отмена)')
            return
        self.resume_after_check = self.state == 'PATROL'
        self.return_state = 'IDLE' if self.state == 'IDLE' else 'STOPPED'
        self.stats['checks'] += 1
        self._event('CHECK нажат в Lichtblick: остановить движение и проверить QR')
        self._request_stop('qr')

    def _has_nav(self):
        return self.goal_future is not None or self.goal_handle is not None

    def _send_next_goal(self):
        if self.state != 'PATROL' or self._has_nav():
            return
        x, y = self.route.points[self.route.index]
        target = PoseStamped()
        target.header.frame_id = 'map'
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.orientation.w = 1.0
        goal = NavigateToPose.Goal()
        goal.pose = target
        self.nav_sent_at = time.monotonic()
        self._event(f'GOAL {self.route.index+1}/{len(self.route.points)}: ({x:.2f},{y:.2f})')
        self.goal_future = self.navigator.send_goal_async(goal)
        self.goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        self.goal_future = None
        try:
            handle = future.result()
        except Exception as exc:
            self._request_stop('nav_error', 'Nav2 goal response: ' + str(exc))
            return
        if not handle.accepted:
            if self.state == 'STOPPING':
                self._complete_stop()  # rejected = no goal executing
            else:
                self._request_stop('nav_error', 'Nav2 отклонил цель')
            return
        self.goal_handle = handle
        self.goal_result_future = handle.get_result_async()
        self.goal_result_future.add_done_callback(self._on_goal_result)
        if self.state != 'PATROL':
            self._cancel_goal()

    def _on_goal_result(self, future):
        try:
            status = future.result().status
        except Exception as exc:
            self.goal_handle = None
            self.goal_result_future = None
            self._force_fault('Неизвестен результат Nav2: ' + str(exc))
            return
        self.goal_handle = None
        self.goal_result_future = None
        self.cancel_future = None
        self.nav_sent_at = None
        if self.state == 'STOPPING':
            if status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED):
                self._complete_stop()
            else:
                self._force_fault('Nav2 не подтвердил безопасное завершение: ' + str(status))
        elif self.state == 'PATROL':
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.stats['goals_reached'] += 1
                _, lap = self.route.advance()
                if lap:
                    self.stats['laps'] += 1
                self._send_next_goal()
            else:
                self._request_stop('nav_error', 'Nav2 завершил цель: ' + str(status))

    def _cancel_goal(self):
        if self.goal_handle is None or self.cancel_future is not None:
            return
        try:
            self.cancel_future = self.goal_handle.cancel_goal_async()
            self.cancel_future.add_done_callback(self._on_cancel_response)
        except Exception as exc:
            self._force_fault('Ошибка отмены Nav2: ' + str(exc))

    def _on_cancel_response(self, future):
        self.cancel_future = None
        try:
            response = future.result()
            if not response.goals_canceling:
                self._event('Nav2 cancel: жду окончательный action result')
        except Exception as exc:
            self._force_fault('Ответ Nav2 cancel: ' + str(exc))

    def _request_stop(self, reason, detail=None):
        if self.state == 'STOPPING':
            if reason in ('reset', 'user_stop', 'fault', 'nav_error'):
                self.stop_reason = reason
                self.error_text = detail
            return
        self.camera.set_mode('off')
        self.stop_reason = reason
        self.error_text = detail
        self.stop_since = time.monotonic()
        self._state('STOPPING', f'Запрос отмены навигации: {reason}')
        if self.goal_handle is not None:
            self._cancel_goal()
        elif self.goal_future is None:
            self._complete_stop()
        # Else: goal request in flight; cancel on acceptance.

    def _complete_stop(self):
        reason = self.stop_reason
        self.stop_reason = None
        self.stop_since = None
        self.nav_sent_at = None
        if reason == 'qr':
            self.qr_camera_deadline = time.monotonic() + QR_CAMERA_TIMEOUT
            self.qr_deadline = None
            self.qr_last_frame_at = None
            self.qr_found = False
            self.qr_text = ''
            self._pub(self.qr_value_pub, '')
            self._qr_status('CHECK: WAITING FOR CAMERA')
            self.camera.set_mode('qr')
            self._state('CHECKING', 'Покажите QR с PERSON_001. На проверку 15 секунд.')
            self._play_sound('patrol_start.wav')
        elif reason == 'reset':
            self.resume_after_check = False
            self.locked = False
            self._stop_audio()
            self.camera.set_mode('off')
            self._qr_status('CAMERA OFF: RESET')
            self._state('IDLE', 'RESET: тревога сброшена; движение не запускается')
        elif reason == 'user_stop':
            self.resume_after_check = False
            self.camera.set_mode('off')
            self._qr_status('CAMERA OFF: STOPPED')
            self._state('STOPPED', 'Остановлен. Камера выключена')
        else:
            self.resume_after_check = False
            self.locked = True
            self.camera.set_mode('off')
            self._state('NAV_ERROR' if reason == 'nav_error' else 'SAFETY_FAULT',
                        self.error_text or 'Не удалось безопасно завершить задачу. Нужен RESET')

    def _force_fault(self, reason):
        self.locked = True
        self.camera.set_mode('off')
        self.resume_after_check = False
        self._stop_audio()
        self._state('SAFETY_FAULT', reason + '. ПРОВЕРЬТЕ ФИЗИЧЕСКУЮ ОСТАНОВКУ')
        if self.goal_handle is not None:
            self._cancel_goal()

    def _on_qr_frame(self, found, code, source_mode):
        # Never authorize a frame produced by PREVIEW before CHECK was pressed.
        if self.state not in ('CHECKING', 'IDLE', 'STOPPED'):
            return
        if source_mode == 'preview' and self.state == 'CHECKING':
            return
        if source_mode == 'qr' and self.state != 'CHECKING':
            return
        now = time.monotonic()
        self.qr_found = bool(found)
        self.qr_text = code
        flag = Bool()
        flag.data = bool(found)
        self.qr_visible_pub.publish(flag)
        self._pub(self.qr_value_pub, code)
        if self.state == 'CHECKING':
            self.qr_last_frame_at = now
            if self.qr_deadline is None:
                self.qr_deadline = now + QR_TIMEOUT
                self._event('QR сканер активен. Отсчёт 15 секунд начался')
            if code:
                outcome = qr_decision(code, ALLOWED_QR)
                if outcome == 'granted':
                    self.stats['granted'] += 1
                    self.camera.set_mode('off')
                    self._qr_status('GRANTED: ' + code)
                    self._play_sound('patrol_success.wav')
                    if self.resume_after_check:
                        self.resume_after_check = False
                        self._state('PATROL', 'QR принят. Продолжаю патруль')
                        self._send_next_goal()
                    else:
                        self._state(self.return_state, 'QR принят; остаюсь на месте')
                elif outcome == 'denied':
                    self._deny('Неверный QR: ' + code)
            elif found:
                self._qr_status('QR DETECTED, UNREADABLE')
        else:
            self._qr_status('PREVIEW: ' + ('QR ' + code if code else 'FOUND / NOT DECODED' if found else 'NO QR'))

    def _deny(self, reason):
        if self.state != 'CHECKING':
            return
        self.stats['denied'] += 1
        self.locked = True
        self.resume_after_check = False
        self.camera.set_mode('off')
        self.qr_deadline = None
        self._qr_status('DENIED: ' + reason)
        self._state('DENIED', reason + '; сирена до RESET')
        self._sound_alarm()

    def _play_sound(self, name):
        self._stop_audio()
        if not shutil.which('aplay'):
            return
        sound = HERE / 'sounds' / name
        if not sound.is_file():
            return
        device = os.environ.get('PATROL_ALSA_DEVICE', 'plughw:0,0')
        try:
            self.audio_process = subprocess.Popen(
                ['aplay', '-q', '-D', device, str(sound)],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError as exc:
            self._event('Звук недоступен: ' + str(exc))

    def _stop_audio(self):
        if self.audio_process is not None and self.audio_process.poll() is None:
            self.audio_process.terminate()
        self.audio_process = None

    def _sound_alarm(self):
        if self.state != 'DENIED':
            return
        if self.audio_process is not None and self.audio_process.poll() is None:
            return
        if time.monotonic() - self.last_sound_try < 2:
            return
        self.last_sound_try = time.monotonic()
        self._play_sound('patrol_siren.wav')

    def _tick(self):
        now = time.monotonic()
        # Bounded drain; publisher always runs on ROS executor, never RTSP thread.
        for _ in range(12):
            try:
                kind, data = self.vision_events.get_nowait()
            except queue.Empty:
                break
            if kind == 'image' and self.camera.mode() in ('qr', 'preview'):
                image = CompressedImage()
                image.header.stamp = self.get_clock().now().to_msg()
                image.header.frame_id = 'camera_optical_frame'
                image.format = 'jpeg'
                image.data = data
                self.image_pub.publish(image)
            elif kind == 'qr_frame':
                self._on_qr_frame(*data)
            elif kind == 'camera_error':
                if self.state == 'CHECKING':
                    self._request_stop('fault', 'QR камера: ' + str(data))
                else:
                    self._qr_status('CAMERA ERROR: ' + str(data))
                    self.camera.set_mode('off')
            elif kind == 'camera_connected':
                self._event('RTSP камера подключена')
        if self.state == 'CHECKING':
            if self.qr_deadline is None and now > self.qr_camera_deadline:
                self._request_stop('fault', 'QR камера не дала кадров за 10 секунд')
            elif self.qr_deadline is not None and now >= self.qr_deadline:
                self._deny('QR не предъявлен за 15 секунд')
            elif self.qr_last_frame_at and now - self.qr_last_frame_at > 6:
                self._request_stop('fault', 'QR камера перестала выдавать кадры')
        if self.state == 'PATROL' and self.nav_sent_at and now - self.nav_sent_at > NAV_GOAL_TIMEOUT:
            self._request_stop('nav_error', 'Навигационная цель слишком долго активна')
        if self.state == 'STOPPING' and self.stop_since and now - self.stop_since > NAV_CANCEL_TIMEOUT:
            self._force_fault('Nav2 не подтвердил завершение отмены за 8 секунд')
        if self.state == 'DENIED':
            self._sound_alarm()

    def destroy_node(self):
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
        self.camera.close()
        self._stop_audio()
        return super().destroy_node()


def main():
    rclpy.init()
    node = Patrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
