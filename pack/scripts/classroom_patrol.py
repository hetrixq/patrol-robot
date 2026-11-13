#!/usr/bin/env python3
import json
import os
from pathlib import Path
import random
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PointStamped, PoseStamped
from slam_toolbox.srv import Reset
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker, MarkerArray

from route_manager import RouteManager
from config import TOPICS

MIN_POINTS = 2
MAX_POINTS = 10
DETECTION_CONFIRM_S = 0.6
QR_TIMEOUT_S = 15.0
QR_PROCESS_GRACE_S = 7.0
DETECTION_COOLDOWN_S = 10.0
DETECTOR_STALE_S = 8.0
NAVIGATION_TIMEOUT_S = 150.0
CANCEL_SETTLE_S = 0.7
STOP_SETTLE_S = 0.8
CANCEL_TIMEOUT_S = 4.0

REPO_ROOT = Path(__file__).resolve().parents[2]
RECOGNITION_DIR = REPO_ROOT / 'robot_recognition'
SIREN = RECOGNITION_DIR / 'sounds' / 'patrol_siren.wav'
DEFAULT_QR_PYTHON = RECOGNITION_DIR / 'venv' / 'bin' / 'python3'


class ClassroomPatrol(RouteManager):
    def __init__(self):
        super().__init__()
        self._lock = threading.RLock()
        self.state = 'IDLE'
        self.needs_reset = False
        self.mode = 'loop'
        self.current_index = 0
        self.running = False
        self.is_paused = True
        self.waiting_for_status = False
        self.confirm_since = None
        self.last_detection_at = None
        self.ignore_detection_until = 0.0
        self.detector_warmup_until = 0.0
        self.last_goal_at = None
        self.resume_at = None
        self.cancel_future = None
        self.cancel_requested_at = None
        self.cancel_issue = None
        self.qr_ready_at = None
        self.session_number = 0
        self.qr_process = None
        self.qr_worker = None
        self.qr_result = None
        self.qr_deadline = None
        self.siren_process = None
        self.next_alarm_try = 0.0
        self.stats = {'granted': 0, 'denied': 0, 'goals_reached': 0, 'laps': 0}
        sticky = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)
        self.state_pub = self.create_publisher(String, '/patrol/state', sticky)
        self.events_pub = self.create_publisher(String, '/patrol/events', 10)
        self.stats_pub = self.create_publisher(String, '/patrol/stats', sticky)
        self.marker_pub = self.create_publisher(MarkerArray, '/patrol/markers', sticky)
        self.verified_pub = self.create_publisher(String, '/person_verifier/result', 10)
        self.create_timer(0.2, self.tick, callback_group=self.cb_group)
        self.create_timer(1.0, self._heartbeat, callback_group=self.cb_group)
        self._announce('IDLE', 'Готов. Отметьте 2–10 точек через Click Point.')
        self._publish_markers()

    def cancel_current_goal(self):
        if self.cancel_future is not None:
            return True
        if not self.cancel_client.service_is_ready():
            self.cancel_issue = 'Сервис /goal/cancel недоступен'
            self.get_logger().error(self.cancel_issue)
            return False
        try:
            self.cancel_future = self.cancel_client.call_async(Reset.Request())
            self.cancel_requested_at = time.monotonic()
            self.cancel_issue = None
            self.get_logger().info('Запрошена отмена /goal/cancel; жду ответ goal_proxy')
            return True
        except Exception as exc:
            self.cancel_issue = f'Ошибка запроса /goal/cancel: {exc}'
            self.get_logger().error(self.cancel_issue)
            return False

    def _cancel_fault(self, reason):
        self.cancel_future = None
        self.cancel_requested_at = None
        self.running = False
        self.is_paused = True
        self.waiting_for_status = False
        self.last_goal_at = None
        self.resume_at = None
        self.qr_ready_at = None
        self._terminate_qr()
        self._stop_siren()
        self._announce('SAFETY_FAULT', reason + '. Проверьте физическую остановку и выполните reset.')

    def _poll_cancel(self, now):
        if self.cancel_future is None:
            return
        if self.cancel_future.done():
            future = self.cancel_future
            self.cancel_future = None
            self.cancel_requested_at = None
            try:
                future.result()  # ответ получен без исключения (Reset имеет пустой response)
            except Exception as exc:
                self._cancel_fault(f'/goal/cancel завершился с ошибкой: {exc}')
                return
            self.get_logger().info('/goal/cancel: ответ получен')
            if self.state == 'STOPPING':
                # Ответ goal_proxy не измеряет скорость колёс. Дадим тормозам паузу.
                self.qr_ready_at = now + STOP_SETTLE_S
        elif self.cancel_requested_at is not None and now - self.cancel_requested_at >= CANCEL_TIMEOUT_S:
            self._cancel_fault('Нет ответа от /goal/cancel за 4 секунды')

    def _emit(self, pub, data):
        msg = String()
        msg.data = data
        pub.publish(msg)

    def _announce(self, state, description):
        self.state = state
        if state in ('DENIED', 'NAV_ERROR', 'SAFETY_FAULT'):
            self.needs_reset = True
        self._emit(self.state_pub, state)
        self._emit(self.events_pub, f'{time.strftime("%H:%M:%S")} {state}: {description}')
        self._emit(self.stats_pub, json.dumps(self.stats, ensure_ascii=False))
        self.get_logger().info(f'[{state}] {description}')

    def _publish_markers(self):
        markers = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)
        for index, point in enumerate(self.waypoints):
            for kind in ('dot', 'label'):
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = f'patrol_{kind}'
                marker.id = index
                marker.action = Marker.ADD
                marker.pose.position.x = float(point.x)
                marker.pose.position.y = float(point.y)
                marker.pose.position.z = 0.12 if kind == 'dot' else 0.44
                marker.pose.orientation.w = 1.0
                marker.color.a = 1.0
                if kind == 'dot':
                    marker.type = Marker.SPHERE
                    marker.scale.x = marker.scale.y = 0.22
                    marker.scale.z = 0.08
                    marker.color.r, marker.color.g, marker.color.b = 0.12, 0.5, 1.0
                else:
                    marker.type = Marker.TEXT_VIEW_FACING
                    marker.scale.z = 0.3
                    marker.color.r = marker.color.g = marker.color.b = 1.0
                    marker.text = str(index + 1)
                markers.markers.append(marker)
        self.marker_pub.publish(markers)

    def click_callback(self, msg):
        with self._lock:
            if self.state not in ('IDLE', 'STOPPED'):
                self.get_logger().warn('Точки можно менять только в IDLE/STOPPED. Нажмите stop/reset.')
                return
            if msg.header.frame_id.lstrip('/') != 'map':
                self.get_logger().warn(f'Click Point должен быть в map, а не {msg.header.frame_id!r}')
                return
            if len(self.waypoints) >= MAX_POINTS:
                self.get_logger().warn(f'Максимум {MAX_POINTS} точек. Используйте undo/clear.')
                return
            self.waypoints.append(msg.point)
            self._publish_markers()
            self.get_logger().info(f'Точка {len(self.waypoints)}: {msg.point.x:.2f}, {msg.point.y:.2f}')

    def command_callback(self, msg):
        command = msg.data.strip().lower()
        with self._lock:
            if command in ('loop', 'loop_on'):
                self.mode = 'loop'
                self._emit(self.events_pub, 'Маршрут: LOOP')
            elif command == 'random':
                self.mode = 'random'
                self._emit(self.events_pub, 'Маршрут: RANDOM')
            elif command == 'undo':
                if self.state in ('IDLE', 'STOPPED') and self.waypoints:
                    self.waypoints.pop()
                    self._publish_markers()
            elif command == 'clear':
                if self.needs_reset:
                    self.get_logger().warn('Сначала reset, потом clear')
                    return
                ok = self._stop_everything()
                if not ok:
                    self._announce('SAFETY_FAULT', self.cancel_issue or 'Не удалось отменить цель')
                    return
                self.waypoints.clear()
                self.current_index = 0
                self._publish_markers()
                self._announce('IDLE', 'Маршрут очищен')
            elif command == 'reset':
                ok = self._stop_everything()
                if not ok:
                    self._announce('SAFETY_FAULT', self.cancel_issue or 'Не удалось отменить цель')
                    return
                self.needs_reset = False
                self.current_index = 0
                self._announce('IDLE', 'Сброс принят. Дождитесь ответа /goal/cancel и отправьте start.')
            elif command == 'stop':
                ok = self._stop_everything()
                if ok:
                    self._announce('STOPPED', 'Ручная остановка; отмена цели отправлена')
                else:
                    self._announce('SAFETY_FAULT', self.cancel_issue or 'Не удалось отменить цель')
            elif command in ('start', 'resume'):
                if self.needs_reset or self.state in ('DENIED', 'NAV_ERROR', 'SAFETY_FAULT'):
                    self.get_logger().warn('Сначала выполните reset!')
                    return
                if self.state == 'PATROL' or self.state == 'CHECKING':
                    return
                if len(self.waypoints) < MIN_POINTS:
                    self.get_logger().warn(f'Нужно минимум {MIN_POINTS} точки')
                    return
                if self.cancel_future is not None:
                    self.get_logger().warn('Отмена предыдущей цели ещё выполняется. Повторите start через секунду.')
                    return
                if not self.cancel_client.service_is_ready():
                    self._announce('SAFETY_FAULT', '/goal/cancel недоступен, патруль запрещён')
                    return
                now = time.monotonic()
                if self.last_detection_at is None or now - self.last_detection_at > DETECTOR_STALE_S:
                    self._announce('SAFETY_FAULT', 'Нет свежих сообщений YOLO /leg_detection/status')
                    return
                self.current_index = (
                    random.randrange(len(self.waypoints)) if self.mode == 'random' and self.state == 'IDLE'
                    else 0 if self.state == 'IDLE'
                    else min(self.current_index, len(self.waypoints) - 1)
                )
                self.running = True
                self.is_paused = False
                self.confirm_since = None
                self._announce('PATROL', f'Старт: {len(self.waypoints)} точек, режим {self.mode}')
                self.enable_detector()
                self.send_next_goal()
            else:
                self.get_logger().warn(f'Неизвестная команда: {command}')

    def _stop_everything(self):
        self.running = False
        self.is_paused = True
        self.waiting_for_status = False
        self.last_goal_at = None
        self.resume_at = None
        self.confirm_since = None
        self.qr_ready_at = None
        self.qr_deadline = None
        self.session_number += 1  # отбрасывает запоздалый результат QR
        self._terminate_qr()
        self._stop_siren()
        self.enable_detector()  # после reset снова нужен свежий heartbeat YOLO
        return self.cancel_current_goal()

    def _terminate_qr(self):
        proc = self.qr_process
        self.qr_process = None
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

    def _stop_siren(self):
        proc = self.siren_process
        self.siren_process = None
        self.next_alarm_try = 0.0
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                proc.kill()

    def _sound_alarm(self):
        if self.state != 'DENIED':
            return
        now = time.monotonic()
        if now < self.next_alarm_try:
            return
        if self.siren_process:
            ret = self.siren_process.poll()
            if ret is None:
                return
            if ret != 0:
                self.next_alarm_try = now + 5.0  # ALSA может быть недоступна
                self.get_logger().error('aplay завершился с ошибкой. Проверить аудиоустройство.')
                return
        if not SIREN.is_file():
            self.next_alarm_try = now + 10.0
            self.get_logger().error(f'Нет файла сирены {SIREN}')
            return
        try:
            self.siren_process = subprocess.Popen(
                ['aplay', '-q', str(SIREN)], stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            self.next_alarm_try = now + 10.0
            self.get_logger().error('Нет aplay: тревога есть в /patrol/state, но нет звука')

    def send_next_goal(self):
        if self.state != 'PATROL' or self.is_paused or not self.waypoints or self.cancel_future is not None:
            return
        point = self.waypoints[self.current_index]
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(point.x)
        goal.pose.position.y = float(point.y)
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.waiting_for_status = True
        self.last_goal_at = time.monotonic()
        self._emit(self.events_pub, f'GOAL {self.current_index+1}/{len(self.waypoints)} ({self.mode})')

    def status_callback(self, msg):
        with self._lock:
            if self.state != 'PATROL' or not self.waiting_for_status:
                return
            status = msg.data.strip().upper()
            if status == 'GOAL SUCCESS':
                self.waiting_for_status = False
                self.last_goal_at = None
                self.stats['goals_reached'] += 1
                old_index = self.current_index
                if self.mode == 'random' and len(self.waypoints) > 1:
                    options = [i for i in range(len(self.waypoints)) if i != old_index]
                    self.current_index = random.choice(options)
                else:
                    self.current_index = (self.current_index + 1) % len(self.waypoints)
                    if self.current_index == 0:
                        self.stats['laps'] += 1
                self._emit(self.stats_pub, json.dumps(self.stats))
                self.send_next_goal()
            elif status in ('GOAL FAILED', 'GOAL FAILURE', 'GOAL ABORTED'):
                self._navigation_error('goal_proxy сообщил ' + status)

    def _navigation_error(self, reason):
        ok = self._stop_everything()
        if ok:
            self._announce('NAV_ERROR', reason + '. Нужен ручной reset')
        else:
            self._announce('SAFETY_FAULT', self.cancel_issue or 'Не удалось остановить робот')

    def detection_status_callback(self, msg):
        with self._lock:
            now = time.monotonic()
            self.last_detection_at = now
            if self.state != 'PATROL' or now < self.ignore_detection_until:
                return
            if msg.data:
                if self.confirm_since is None:
                    self.confirm_since = now
                elif now - self.confirm_since >= DETECTION_CONFIRM_S:
                    self._begin_qr()
            else:
                self.confirm_since = None

    def _begin_qr(self):
        self.is_paused = True
        self.waiting_for_status = False
        self.last_goal_at = None
        self.confirm_since = None
        if not self.cancel_current_goal():
            self._cancel_fault(self.cancel_issue or 'Не удалось отменить цель перед проверкой')
            return
        self.disable_detector()  # освобождаем CPU для чтения QR
        self._announce('STOPPING', 'Человек обнаружен. Останавливаю робот перед проверкой QR.')

    def _start_qr_session(self):
        if self.state != 'STOPPING':
            return
        self.qr_ready_at = None
        self._announce('CHECKING', f'Навигационная цель отменена. Покажите QR за {QR_TIMEOUT_S:.0f} секунд.')
        self.session_number += 1
        session = self.session_number
        self.qr_deadline = time.monotonic() + QR_TIMEOUT_S
        self.qr_result = None
        self.qr_worker = threading.Thread(target=self._qr_job, args=(session,), daemon=True)
        self.qr_worker.start()

    def _qr_job(self, session):
        python_bin = os.environ.get('PATROL_QR_PYTHON') or (
            str(DEFAULT_QR_PYTHON) if DEFAULT_QR_PYTHON.is_file() else sys.executable)
        try:
            proc = subprocess.Popen(
                [python_bin, 'qr_only.py'], cwd=str(RECOGNITION_DIR),
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                start_new_session=True)
            with self._lock:
                if session != self.session_number:
                    os.killpg(proc.pid, signal.SIGTERM)
                    return
                self.qr_process = proc
            try:
                output, _ = proc.communicate(timeout=QR_TIMEOUT_S + QR_PROCESS_GRACE_S)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                output, _ = proc.communicate()
                outcome = {'result': 'fall', 'reason': 'qr_process_timeout'}
            else:
                outcome = {'result': 'fall', 'reason': 'qr_process_failed'}
                for line in output.splitlines():
                    try:
                        parsed = json.loads(line)
                    except ValueError:
                        continue
                    if isinstance(parsed, dict) and parsed.get('result') in ('success', 'fall', 'error'):
                        outcome = parsed
            self.get_logger().info('QR output: ' + output[-800:])
        except Exception as exc:
            outcome = {'result': 'fall', 'reason': f'launch_error: {exc}'}
        with self._lock:
            if session == self.session_number:
                self.qr_result = outcome
                self.qr_process = None

    def _finish_qr(self, result):
        self.qr_result = None
        self.qr_deadline = None
        self.session_number += 1  # поздние результаты подпроцесса игнорируем
        self._terminate_qr()
        if result.get('result') == 'error' or result.get('reason') in ('launch_error', 'qr_process_timeout', 'qr_process_failed', 'scanner_stopped') or str(result.get('reason', '')).startswith('launch_error'):
            self.running = False
            self.is_paused = True
            self._announce('SAFETY_FAULT', f'Неисправность QR-распознавателя: {result.get("reason")}. Требуется reset.')
        elif result.get('result') == 'success':
            self.stats['granted'] += 1
            now = time.monotonic()
            self.ignore_detection_until = now + DETECTION_COOLDOWN_S
            self.detector_warmup_until = now + DETECTOR_STALE_S
            self.last_detection_at = now
            self._emit(self.verified_pub, 'success')  # существующий cooldown leg_detector
            self.enable_detector()
            self.is_paused = False
            self._announce('PATROL', 'QR принят. Продолжаю движение.')
            self.resume_at = now + CANCEL_SETTLE_S
        else:
            self.stats['denied'] += 1
            self.running = False
            self.is_paused = True
            self._announce('DENIED', 'Нет правильного QR за 15 секунд. Ожидание reset.')
            self._sound_alarm()

    def tick(self):
        with self._lock:
            now = time.monotonic()
            self._poll_cancel(now)
            if self.state == 'STOPPING' and self.qr_ready_at is not None and now >= self.qr_ready_at:
                self._start_qr_session()
            if self.state == 'CHECKING':
                if self.qr_result is not None:
                    self._finish_qr(self.qr_result)
                elif self.qr_deadline is not None and now >= self.qr_deadline:
                    self._finish_qr({'result': 'fall', 'reason': '15_seconds_elapsed'})
            if self.state == 'PATROL':
                if self.resume_at is not None and now >= self.resume_at and self.cancel_future is None:
                    self.resume_at = None
                    self.send_next_goal()  # повторяем прерванную цель, не перепрыгиваем
                if now >= self.detector_warmup_until and (
                    self.last_detection_at is None or now-self.last_detection_at > DETECTOR_STALE_S
                ):
                    self._stop_everything()
                    self._announce('SAFETY_FAULT', 'YOLO/камера не присылает статус. Нужен reset.')
                elif self.waiting_for_status and self.last_goal_at is not None and (
                    now-self.last_goal_at > NAVIGATION_TIMEOUT_S
                ):
                    self._navigation_error('Таймаут движения к точке')
                elif not self.cancel_client.service_is_ready():
                    self._stop_everything()
                    self._announce('SAFETY_FAULT', '/goal/cancel пропал. Нужен reset.')
            if self.state == 'DENIED':
                self._sound_alarm()

    def _heartbeat(self):
        with self._lock:
            self._emit(self.state_pub, self.state)
            self._emit(self.stats_pub, json.dumps(self.stats, ensure_ascii=False))

    def destroy_node(self):
        with self._lock:
            self._stop_everything()
        super().destroy_node()


def main():
    rclpy.init()
    node = ClassroomPatrol()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
