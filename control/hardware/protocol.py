"""Explicit interface between trajectory controllers and hardware."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol, Sequence
import numpy as np


@dataclass(frozen=True)
class HardwareState:
    """One timestamped hardware sample in SI units."""

    time_s: float
    joint_position_rad: np.ndarray
    joint_velocity_rad_s: np.ndarray
    tcp_pose_m: np.ndarray
    wrench_N_m: np.ndarray = field(default_factory=lambda: np.zeros(6, dtype=float))
    connected: bool = True


class HardwareBackend(Protocol):
    """Minimal lifecycle and command contract for position/force control."""

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def read_state(self) -> HardwareState: ...
    def read_force(self) -> np.ndarray: ...
    def command_joint_position(self, position_rad: Sequence[float]) -> None: ...
    def command_joint_velocity(self, velocity_rad_s: Sequence[float]) -> None: ...
    def emergency_stop(self, reason: str) -> None: ...
