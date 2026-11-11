"""Saved-trajectory playback clock, waypoint stepping, and GIF time sampling."""
from dataclasses import dataclass
import numpy as np
from ..infrastructure.trajectory import Trajectory


@dataclass
class Playback:
    trajectory: Trajectory
    speed: float = 5.
    time_s: float = 0.
    paused: bool = False

    def __post_init__(self):
        if not np.isfinite(self.speed) or self.speed <= 0:
            raise ValueError('Playback speed must be positive and finite')
        self.seek(self.time_s)

    def seek(self, time_s):
        if not np.isfinite(time_s) or not 0 <= time_s <= self.trajectory.duration:
            raise ValueError(f'Time must be between 0 and {self.trajectory.duration:g} seconds')
        self.time_s = float(time_s)

    def advance(self, elapsed_s):
        if not np.isfinite(elapsed_s) or elapsed_s < 0:
            raise ValueError('Elapsed time must be nonnegative and finite')
        if not self.paused:
            self.time_s = (self.time_s+elapsed_s*self.speed) % self.trajectory.duration
        return self.trajectory.sample(self.time_s)

    def step(self, direction):
        times = self.trajectory.times
        if direction > 0:
            index = np.searchsorted(times, self.time_s+1e-9, side='right')
        else:
            index = np.searchsorted(times, self.time_s-1e-9, side='left')-1
        self.time_s = float(times[int(np.clip(index, 0, len(times)-1))])
        self.paused = True

    def key(self, code):
        if code == 32:
            self.paused = not self.paused
        elif code in (ord('N'), ord('P')):
            self.step(1 if code == ord('N') else -1)
        elif code == ord('R'):
            self.time_s = 0.


def animation_times(duration_s, speed, fps):
    """Include both endpoints while rounding GIF delays to its 10 ms resolution."""
    if not all(np.isfinite(x) and x > 0 for x in (duration_s, speed, fps)):
        raise ValueError('Duration, speed and frame rate must be positive and finite')
    if fps > 50:
        raise ValueError('GIF frame rate must not exceed 50 fps')
    delay_ms = max(20, round(100./fps)*10)
    count = max(2, int(np.ceil(duration_s/speed/(delay_ms*.001))))
    if count > 3000:
        raise ValueError('GIF exceeds 3000 frames; increase --speed or reduce --fps')
    return np.linspace(0., duration_s, count), delay_ms
