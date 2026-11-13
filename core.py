"""Pure-Python route and QR decisions; no ROS, camera, or GPU imports."""
from dataclasses import dataclass, field
import random

MIN_POINTS = 2
MAX_POINTS = 10


@dataclass
class Route:
    points: list = field(default_factory=list)
    index: int = 0
    mode: str = 'loop'

    def add(self, x, y):
        if len(self.points) >= MAX_POINTS:
            raise ValueError('maximum 10 waypoints')
        self.points.append((float(x), float(y)))

    def undo(self):
        if self.points:
            self.points.pop()
            self.index = min(self.index, max(0, len(self.points) - 1))

    def clear(self):
        self.points.clear()
        self.index = 0

    def set_mode(self, mode):
        if mode not in ('loop', 'random'):
            raise ValueError('mode must be loop or random')
        self.mode = mode

    def ready(self):
        return MIN_POINTS <= len(self.points) <= MAX_POINTS

    def advance(self, rng=None):
        if not self.points:
            raise ValueError('route empty')
        old = self.index
        if self.mode == 'random' and len(self.points) > 1:
            choices = [i for i in range(len(self.points)) if i != old]
            self.index = (rng or random).choice(choices)
        else:
            self.index = (old + 1) % len(self.points)
        return self.index, (self.mode == 'loop' and self.index == 0)


def qr_decision(decoded_value: str, allowed_code: str = 'PERSON_001') -> str:
    """Returns granted/denied; no decoded value means continue scanning."""
    if not decoded_value:
        return 'wait'
    return 'granted' if decoded_value.strip() == allowed_code else 'denied'
