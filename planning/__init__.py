"""Pose selection and robot motion planning from perception regions.

The public planning stages consume perception output through ``pose_search`` and
``motion_planning``. Robot kinematics, collision checking and trajectory
serialization live in the shared ``infrastructure`` package.
"""

__all__ = ["pose_search", "motion_planning"]
