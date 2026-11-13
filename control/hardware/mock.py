"""Deterministic backend for controller tests and module demos."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .protocol import HardwareState


@dataclass
class MockHardwareBackend:
    joint_count: int = 6
    dt_s: float = 0.001

    def __post_init__(self):
        if self.joint_count <= 0 or self.dt_s <= 0:
            raise ValueError('joint_count and dt_s must be positive')
        self.position = np.zeros(self.joint_count, dtype=float)
        self.velocity = np.zeros(self.joint_count, dtype=float)
        self.force = np.zeros(6, dtype=float)
        self.time_s = 0.0
        self.connected = False
        self.stopped = False
        self.stop_reason = ''

    def connect(self):
        self.connected = True
        self.stopped = False

    def disconnect(self):
        self.connected = False

    def _require_connected(self):
        if not self.connected:
            raise RuntimeError('Mock hardware is not connected')
        if self.stopped:
            raise RuntimeError(f'Mock hardware is stopped: {self.stop_reason}')

    def read_state(self):
        self._require_connected()
        return HardwareState(self.time_s, self.position.copy(), self.velocity.copy(),
                             np.eye(4), self.force.copy(), True)

    def read_force(self):
        return self.read_state().wrench_N_m

    def command_joint_position(self, position_rad):
        self._require_connected()
        target = np.asarray(position_rad, dtype=float)
        if target.shape != self.position.shape or not np.isfinite(target).all():
            raise ValueError('joint position shape or values are invalid')
        self.velocity = (target - self.position) / self.dt_s
        self.position = target.copy()
        self.time_s += self.dt_s

    def command_joint_velocity(self, velocity_rad_s):
        self._require_connected()
        velocity = np.asarray(velocity_rad_s, dtype=float)
        if velocity.shape != self.velocity.shape or not np.isfinite(velocity).all():
            raise ValueError('joint velocity shape or values are invalid')
        self.velocity = velocity.copy()
        self.position += velocity * self.dt_s
        self.time_s += self.dt_s

    def emergency_stop(self, reason):
        self.stopped = True
        self.stop_reason = str(reason)
        self.velocity.fill(0.)
