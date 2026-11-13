"""Measured-motion guard adapted from robot_clean's standalone force_motion logic."""
import math


class MotionProgressGuard:
    def __init__(self, timeout=1., min_command=.0001, min_progress=.00002):
        if min(timeout,min_command,min_progress)<=0:
            raise ValueError('Progress guard limits must be positive')
        self.timeout,self.min_command,self.min_progress=timeout,min_command,min_progress
        self.start_time=self.start_position=None
        self.direction=0

    def update(self, position, command, now):
        if not all(math.isfinite(v) for v in (position,command,now)):
            raise ValueError('Invalid motion feedback')
        direction=0 if abs(command)<self.min_command else (1 if command>0 else -1)
        if self.start_time is not None and now<self.start_time:
            raise ValueError('Motion feedback time went backwards')
        if direction!=self.direction or self.start_time is None:
            self.direction=direction;self.start_time=now;self.start_position=position
            return
        if direction and now-self.start_time>=self.timeout:
            if direction*(position-self.start_position)<self.min_progress:
                raise RuntimeError('Commanded normal motion has no measured pose progress')
            self.start_time=now;self.start_position=position
