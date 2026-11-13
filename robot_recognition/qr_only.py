#!/usr/bin/env python3
import json
import queue
import sys
import threading
import time

from card_recognizer import CardRecognizer
from audio_player import play_sound

TIMEOUT_S = 15.0


def safe_sound(name):
    try:
        play_sound(name, blocking=False)
    except Exception as exc:
        print(f'[qr_only] звук недоступен: {exc}', file=sys.stderr)


def run_session():
    events = queue.Queue()
    stopped = threading.Event()
    recognizer = CardRecognizer(on_detected=events.put, stop_event=stopped)
    recognizer.start()
    safe_sound('patrol_start.wav')
    deadline = time.monotonic() + TIMEOUT_S
    result = {'result': 'fall', 'reason': 'timeout'}
    try:
        while time.monotonic() < deadline:
            if not recognizer.is_alive():
                result = {'result': 'error', 'reason': 'scanner_stopped'}
                break
            try:
                code = events.get(timeout=min(0.2, max(0.001, deadline-time.monotonic())))
            except queue.Empty:
                continue
            result = {'result': 'success', 'source': 'card', 'value': code}
            break
    finally:
        stopped.set()
        recognizer.join(timeout=0.3)  # поток daemon, не блокируем ROS
    if result['result'] == 'success':
        safe_sound('patrol_success.wav')
    print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0 if result['result'] == 'success' else 1


if __name__ == '__main__':
    sys.exit(run_session())
