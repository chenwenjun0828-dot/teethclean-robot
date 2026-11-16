"""Hardware boundary shared by simulation, mock, and future robot drivers."""

from .protocol import HardwareBackend, HardwareState
from .mock import MockHardwareBackend
from .ur_robot import URRobotBackend


def __getattr__(name):
    if name == 'MujocoHardwareBackend':
        from .simulation import MujocoHardwareBackend
        return MujocoHardwareBackend
    raise AttributeError(name)

__all__ = [
    'HardwareBackend',
    'HardwareState',
    'MockHardwareBackend',
    'MujocoHardwareBackend',
    'URRobotBackend',
]
