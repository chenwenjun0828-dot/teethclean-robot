"""Reserved Universal Robots adapter boundary.

The transport is intentionally injected by a future deployment package so
simulation code cannot accidentally open a real robot connection.
"""

from __future__ import annotations

import numpy as np


class URRobotBackend:
    def __init__(self, host, *, transport=None, joint_count=6):
        self.host = str(host)
        self.transport = transport
        self.joint_count = int(joint_count)
        if self.joint_count <= 0:
            raise ValueError('joint_count must be positive')

    def connect(self):
        if self.transport is None:
            raise RuntimeError('No real-robot transport configured for URRobotBackend')
        return self.transport.connect(self.host)

    def disconnect(self):
        if self.transport is not None:
            return self.transport.disconnect()

    def read_state(self):
        if self.transport is None:
            raise RuntimeError('No real-robot transport configured for URRobotBackend')
        return self.transport.read_state()

    def read_force(self):
        if self.transport is None:
            raise RuntimeError('No real-robot transport configured for URRobotBackend')
        return np.asarray(self.transport.read_force(), dtype=float)

    def command_joint_position(self, position_rad):
        self._command('command_joint_position', position_rad)

    def command_joint_velocity(self, velocity_rad_s):
        self._command('command_joint_velocity', velocity_rad_s)

    def _command(self, name, values):
        if self.transport is None:
            raise RuntimeError('No real-robot transport configured for URRobotBackend')
        values = np.asarray(values, dtype=float)
        if values.shape != (self.joint_count,) or not np.isfinite(values).all():
            raise ValueError(f'{name} expects {self.joint_count} finite joint values')
        return getattr(self.transport, name)(values)

    def emergency_stop(self, reason):
        if self.transport is not None:
            return self.transport.emergency_stop(str(reason))
