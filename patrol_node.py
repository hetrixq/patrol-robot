#!/usr/bin/env python3
"""RBM-native, low-load classroom patrol.

Uses RBM's EXISTING SLAM, Nav2 NavigateToPose action, MediaMTX, and Lichtblick
bridge. No goal_proxy cancellation ambiguity, no cloned upstream nodes, no
second camera decoder. This is educational software, not a safety controller.
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
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker, MarkerArray

from core import Route, qr_decision
from vision import VisionWorker

DIR = Path(__file__).resolve().parent
QR_TIMEOUT = 15.0
CAMERA_START_TIMEOUT = 30.0
QR_CAMERA_TIMEOUT = 10.0
NAV_CANCEL_TIMEOUT = 6.0
NAV_GOAL_TIMEOUT = 120.0
VISION_STALE_TIMEOUT = 5.0
ALLOWED_QR = os.environ.get('PATROL_ALLOWED_QR', 'PERSON_001')


class Patrol(Node):
    def __init__(self):
        super().__init__('patrol_classroom_v4')
        self.route = Route()
        self.state = 'IDLE'
        self.locked = False
        self.detect_count = 0
        self.clear_count = 0
        self.cooldown_until = 0.0
        self.last_vision = None
        self.start_deadline = None
        self.qr_camera_deadline = None
        self.qr_deadline = None
        self.nav_sent_at = None
        self.stop_since = None
        self.stop_reason = None
        self.error_text = None
        self.goal_future = None
        self.goal_handle = None
        self.goal_result_future = None
        self.cancel_future = None
        self.siren = None
        self.audio_process = None
        self.last_sound_try = 0.0
        self.stats = {'granted': 0, 'denied': 0, 'goals_reached': 0, 'laps': 0}
        self.vision_events = queue.Queue(maxsize=48)
        self.camera = VisionWorker(
            self.vision_events,
            url=os.environ.get('PATROL_RTSP_URL', 'rtsp://localhost:8554/cam'),
            model_path=os.environ.get('PATROL_MODEL', '/root/weights/yolo11n_ncnn_model'),
            fps=float(os.environ.get('PATROL_YOLO_FPS', '1.5')),
            qr_fps=float(os.environ.get('PATROL_QR_FPS', '3')),
            imgsz=int(os.environ.get('PATROL_IMGSZ', '320')),
            conf=float(os.environ.get('PATROL_CONF', '0.45')),
        )
        sticky = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/patrol/state', sticky)
        self.events_pub = self.create_publisher(String, '/patrol/events', 10)
        self.stats_pub = self.create_publisher(String, '/patrol/stats', sticky)
        self.markers_pub = self.create_publisher(MarkerArray, '/patrol/markers', sticky)
        self.person_pub = self.create_publisher(Bool, '/patrol/person_detected', 10)
        self.vision_pub = self.create_publisher(String, '/patrol/vision', sticky)
        self.create_subscription(PointStamped, '/clicked_point', self._clicked, 10)
        self.create_subscription(String, '/robot_commands', self._command, 10)
        self.navigator = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.create_timer(0.2, self._tick)
        self.create_timer(1.0, self._heartbeat)
        self._publish_markers()
        self._state('IDLE', 'Готов. В Lichtblick поставьте 2–10 точек; YOLO выключена.')

    def _pub(self, publisher, value):
        msg = String()
        msg.data = value
        publisher.publish(msg)

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
        self._pub(self.vision_pub, f'{self.camera.mode()} (last YOLO: '
                  f'{"none" if self.last_vision is None else round(time.monotonic()-self.last_vision,1)} s ago)')

    def _publish_markers(self):
        msg = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        msg.markers.append(clear)
        for index, (x, y) in enumerate(self.route.points):
            for kind in ('dot', 'label'):
                mark = Marker()
                mark.header.frame_id = 'map'
                mark.header.stamp = self.get_clock().now().to_msg()
                mark.ns = 'patrol_' + kind
                mark.id = index
                mark.action = Marker.ADD
                mark.pose.position.x, mark.pose.position.y = x, y
                mark.pose.position.z = 0.12 if kind == 'dot' else 0.45
                mark.pose.orientation.w = 1.0
                mark.color.a = 1.0
                if kind == 'dot':
                    mark.type = Marker.SPHERE
                    mark.scale.x = mark.scale.y = 0.22
                    mark.scale.z = 0.08
                    mark.color.r, mark.color.g, mark.color.b = 0.1, 0.5, 1.0
                else:
                    mark.type = Marker.TEXT_VIEW_FACING
                    mark.scale.z = 0.32
                    mark.color.r = mark.color.g = mark.color.b = 1.0
                    mark.text = str(index + 1)
                msg.markers.append(mark)
        self.markers_pub.publish(msg)

    def _clicked(self, msg):
        if self.state not in ('IDLE', 'STOPPED') or self.locked or self._has_nav():
            self._event('Точки изменяются только в IDLE/STOPPED без активной цели')
            return
        if msg.header.frame_id.lstrip('/') != 'map':
            self._event('Click Point должен публиковаться в map')
            return
        try:
            self.route.add(msg.point.x, msg.point.y)
        except ValueError as exc:
            self._event(str(exc))
            return
        self._publish_markers()
        self._event(f'Точка {len(self.route.points)}: {msg.point.x:.2f} {msg.point.y:.2f}')

    def _command(self, msg):
        cmd = msg.data.strip().lower()
        if cmd in ('loop', 'random'):
            if self.state in ('IDLE', 'STOPPED'):
                self.route.set_mode(cmd)
                self._event('Режим маршрута: ' + cmd)
            return
        if cmd in ('clear', 'undo'):
            if self.state not in ('IDLE', 'STOPPED') or self.locked or self._has_nav():
                self._event('Сначала STOP или RESET; движение должно завершиться')
                return
            if cmd == 'clear':
                self.route.clear()
            else:
                self.route.undo()
            self._publish_markers()
            return
        if cmd in ('start', 'resume'):
            self._start()
        elif cmd == 'stop':
            if self.state == 'DENIED':
                self._event('DENIED: для отключения сирены и снятия блокировки нажмите RESET')
            elif self.state != 'STOPPING':
                self._request_stop('user_stop')
        elif cmd == 'reset':
            self._request_stop('reset')
        else:
            self._event('Неизвестная команда ' + cmd)

    def _start(self):
        if self.locked or self.state not in ('IDLE', 'STOPPED') or self._has_nav():
            self._event('START недоступен. Нужен RESET или завершение отмены.')
            return
        if not self.route.ready():
            self._event('Для запуска требуется 2–10 точек')
            return
        if not self.navigator.server_is_ready():
            self._event('Nav2 action /navigate_to_pose ещё недоступен')
            return
        self.detect_count = 0
        self.clear_count = 0
        self.last_vision = None
        self.start_deadline = time.monotonic() + CAMERA_START_TIMEOUT
        self.camera.set_mode('person')
        self._state('STARTING', 'Включаю камеру/YOLO, ещё НЕ двигаюсь')

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
        self._event(f'GOAL {self.route.index + 1}/{len(self.route.points)}: ({x:.2f}, {y:.2f})')
        self.goal_future = self.navigator.send_goal_async(goal)
        self.goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        self.goal_future = None
        try:
            handle = future.result()
        except Exception as exc:
            self._nav_response_error(str(exc))
            return
        if not handle.accepted:
            self._nav_response_error('Nav2 отклонил цель')
            return
        self.goal_handle = handle
        self.goal_result_future = handle.get_result_async()
        self.goal_result_future.add_done_callback(self._on_goal_result)
        if self.state != 'PATROL':
            # STOPPING/SAFETY_FAULT may have happened while request was in flight.
            # Never leave a late-accepted Nav2 goal moving the robot.
            self._cancel_goal()

    def _nav_response_error(self, reason):
        if self.state == 'STOPPING':
            self._complete_stop()  # цель не принята — нечего отменять
        else:
            self._request_stop('nav_error', reason)

    def _on_goal_result(self, future):
        try:
            status = future.result().status
        except Exception as exc:
            self.goal_handle = None
            self.goal_result_future = None
            self._request_stop('nav_error', 'Ошибка результата Nav2: ' + str(exc))
            return
        self.goal_handle = None
        self.goal_result_future = None
        self.cancel_future = None
        self.nav_sent_at = None
        if self.state == 'STOPPING':
            if status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED):
                self._complete_stop()
            else:
                self._force_fault(f'Nav2 не подтвердил отмену, статус={status}')
        elif self.state == 'PATROL':
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.stats['goals_reached'] += 1
                _, lap = self.route.advance()
                if lap:
                    self.stats['laps'] += 1
                self._send_next_goal()
            else:
                self._request_stop('nav_error', f'Nav2 завершил цель со статусом {status}')

    def _cancel_goal(self):
        if self.goal_handle is None or self.cancel_future is not None:
            return
        try:
            self.cancel_future = self.goal_handle.cancel_goal_async()
            self.cancel_future.add_done_callback(self._on_cancel_response)
        except Exception as exc:
            self._force_fault('Не удалось запросить отмену Nav2: ' + str(exc))

    def _on_cancel_response(self, future):
        self.cancel_future = None
        try:
            response = future.result()
            if not response.goals_canceling:
                # A goal may already have finished. Wait for get_result_async,
                # never assume the robot physically stopped from this response.
                self._event('Nav2 не включил цель в goals_canceling; жду результат action')
        except Exception as exc:
            self._force_fault('Ошибка ответа cancel_goal_async: ' + str(exc))

    def _request_stop(self, reason, detail=None):
        # RESET never bypasses a still-active navigation goal.
        if self.state == 'STOPPING':
            if reason in ('reset', 'user_stop', 'fault', 'nav_error'):
                self.stop_reason = reason
                self.error_text = detail
            return
        self.camera.set_mode('hold' if reason == 'qr' else 'off')
        self.stop_reason = reason
        self.error_text = detail
        self.stop_since = time.monotonic()
        self._state('STOPPING', f'Отменяю Nav2: {reason}; новые цели запрещены')
        if self.goal_handle is not None:
            self._cancel_goal()
        elif self.goal_future is None:
            self._complete_stop()
        # else: goal request in flight; cancel upon acceptance.

    def _complete_stop(self):
        reason = self.stop_reason
        self.stop_reason = None
        self.stop_since = None
        self.nav_sent_at = None
        self.detect_count = 0
        if reason == 'qr':
            self.qr_camera_deadline = time.monotonic() + QR_CAMERA_TIMEOUT
            self.qr_deadline = None
            self.camera.set_mode('qr')
            self._state('CHECKING', 'QR-камера запускается; ждём первый кадр')
            self._play_sound('patrol_start.wav')
        elif reason == 'reset':
            self.locked = False
            self._stop_audio()
            self.camera.set_mode('off')
            self._state('IDLE', 'RESET принят; камера выключена')
        elif reason == 'user_stop':
            self.camera.set_mode('off')
            self._state('STOPPED', 'Остановлен. YOLO выключена')
        else:
            self.locked = True
            self.camera.set_mode('off')
            self._state('NAV_ERROR' if reason == 'nav_error' else 'SAFETY_FAULT',
                        self.error_text or 'Сбой. Нужен RESET и проверка физической остановки')

    def _force_fault(self, reason):
        self.locked = True
        self.camera.set_mode('off')
        self._stop_audio()
        # Any unresolved Nav2 goal remains a physical hazard.
        self._state('SAFETY_FAULT', reason + '. ПРОВЕРЬТЕ ФИЗИЧЕСКУЮ ОСТАНОВКУ')
        if self.goal_handle is not None:
            self._cancel_goal()  # best effort; user must physically stop if Nav2 does not

    def _begin_check(self):
        self._request_stop('qr')

    def _on_person(self, detected):
        now = time.monotonic()
        self.last_vision = now
        report = Bool()
        report.data = detected
        self.person_pub.publish(report)
        if self.state == 'STARTING':
            if detected:
                self.clear_count = 0
                if now >= self.cooldown_until:
                    self.detect_count += 1
                    if self.detect_count >= 2:
                        self._begin_check()
            else:
                self.detect_count = 0
                self.clear_count += 1
                if self.clear_count >= 2:
                    self._state('PATROL', 'YOLO готова, 2 чистых кадра, начинаю маршрут')
                    self._send_next_goal()
        elif self.state == 'PATROL':
            if now < self.cooldown_until:
                return
            self.detect_count = self.detect_count + 1 if detected else 0
            if self.detect_count >= 2:
                self._begin_check()

    def _on_qr_code(self, value):
        if self.state != 'CHECKING' or self.qr_deadline is None:
            return
        outcome = qr_decision(value, ALLOWED_QR)
        if outcome == 'granted':
            self.stats['granted'] += 1
            self.camera.set_mode('off')
            self._play_sound('patrol_success.wav')
            self.cooldown_until = time.monotonic() + 10.0
            self.last_vision = None
            self.clear_count = 0
            self.detect_count = 0
            self.start_deadline = time.monotonic() + CAMERA_START_TIMEOUT
            self.camera.set_mode('person')
            self._state('STARTING', 'QR принят, восстанавливаю YOLO перед продолжением')
        elif outcome == 'denied':
            self._deny('Показан неправильный QR')

    def _deny(self, reason):
        self.stats['denied'] += 1
        self.locked = True
        self.camera.set_mode('off')
        self.qr_deadline = None
        self._state('DENIED', reason + '; сирена до RESET')
        self._sound_alarm()

    def _play_sound(self, name):
        self._stop_audio()
        if not shutil.which('aplay'):
            return
        sound = DIR / 'sounds' / name
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
        if time.monotonic() - self.last_sound_try < 2.0:
            return
        self.last_sound_try = time.monotonic()
        self._play_sound('patrol_siren.wav')

    def _tick(self):
        now = time.monotonic()
        # Drain quickly, but do not allow an unbounded stream of camera events
        # to block ROS navigation callbacks.
        for _ in range(20):
            try:
                kind, data = self.vision_events.get_nowait()
            except queue.Empty:
                break
            if kind == 'person' and self.state in ('STARTING', 'PATROL'):
                self._on_person(data)
            elif kind == 'qr_frame' and self.state == 'CHECKING':
                if self.qr_deadline is None:
                    self.qr_deadline = now + QR_TIMEOUT
                    self._event('Покажите QR. Отсчёт 15 секунд начался')
            elif kind == 'qr_code':
                self._on_qr_code(data)
            elif kind in ('vision_error', 'camera_error'):
                if self.state in ('STARTING', 'PATROL', 'CHECKING'):
                    self._request_stop('fault', str(data))
            elif kind == 'model_ready':
                self._event('YOLO11n NCNN загружена')
            elif kind == 'camera_connected':
                self._event('RTSP поток подключён')

        if self.state == 'STARTING':
            if self.last_vision is None and self.start_deadline and now > self.start_deadline:
                self._request_stop('fault', 'Не получен первый результат YOLO за 30 с')
            elif self.last_vision is not None and now - self.last_vision > VISION_STALE_TIMEOUT:
                self._request_stop('fault', 'Детектор перестал выдавать кадры при запуске')
        if self.state == 'PATROL':
            if self.last_vision is None or now - self.last_vision > VISION_STALE_TIMEOUT:
                self._request_stop('fault', 'Нет свежих результатов YOLO')
            elif self.nav_sent_at and now - self.nav_sent_at > NAV_GOAL_TIMEOUT:
                self._request_stop('nav_error', 'Слишком долго едем к точке')
        if self.state == 'CHECKING':
            if self.qr_deadline is None and now > self.qr_camera_deadline:
                self._request_stop('fault', 'Не удалось получить кадры QR-камеры')
            elif self.qr_deadline is not None and now >= self.qr_deadline:
                self._deny('QR не предъявлен за 15 секунд')
        if self.state == 'STOPPING' and self.stop_since and now - self.stop_since > NAV_CANCEL_TIMEOUT:
            self._force_fault('Навигационная цель не подтвердила остановку за 6 с')
        if self.state == 'DENIED':
            self._sound_alarm()

    def destroy_node(self):
        # A stopped node cannot control motors: send direct Nav2 cancel if
        # possible, but actual motor safety requires physical stop.
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
