"""Pair-by-pair landmark state, independent of the rendering/UI backend."""
from __future__ import annotations
import operator
import numpy as np


class PairSelection:
    def __init__(self, scan_points, camera_points, minimum_pairs=6):
        self.scan_points = self._points(scan_points)
        self.camera_points = self._points(camera_points)
        if isinstance(minimum_pairs, bool) or not isinstance(minimum_pairs, int) or minimum_pairs < 3:
            raise ValueError("At least three complete pairs are required")
        self.minimum_pairs = minimum_pairs
        self.scan_indices = []
        self.camera_indices = []
        self.pending_camera = None

    @staticmethod
    def _points(value):
        value = np.asarray(value, dtype=float)
        if value.ndim != 2 or value.shape[1] != 3 or not len(value) or not np.isfinite(value).all():
            raise ValueError("Selection requires a nonempty finite Nx3 point cloud")
        return value.copy()

    @property
    def pair_count(self):
        return len(self.scan_indices)

    @property
    def next_side(self):
        return "camera" if self.pending_camera is None else "scan"

    @property
    def can_finish(self):
        return self.pair_count >= self.minimum_pairs and self.pending_camera is None

    def select(self, side, index):
        if side != self.next_side:
            raise ValueError("Pick the LEFT camera point first" if self.next_side == "camera"
                             else "Pick the matching RIGHT scan point to complete this pair")
        try:
            if isinstance(index, (bool, np.bool_)):
                raise TypeError()
            index = operator.index(index)
        except TypeError:
            raise ValueError("Point index must be an integer") from None
        points = self.camera_points if side == "camera" else self.scan_points
        used = self.camera_indices if side == "camera" else self.scan_indices
        if not 0 <= index < len(points):
            raise ValueError("Point index is outside the point cloud")
        if index in used:
            raise ValueError("This point is already selected; choose another landmark")
        if side == "camera":
            self.pending_camera = index
        else:
            self.camera_indices.append(self.pending_camera)
            self.scan_indices.append(index)
            self.pending_camera = None

    def undo(self):
        if self.pending_camera is not None:
            self.pending_camera = None
        elif self.scan_indices:
            self.scan_indices.pop()
            self.camera_indices.pop()

    def clear(self):
        self.scan_indices.clear()
        self.camera_indices.clear()
        self.pending_camera = None

    def result(self):
        if self.pending_camera is not None:
            raise ValueError("Complete the pending pair or undo the LEFT point")
        if not self.can_finish:
            raise ValueError(f"At least {self.minimum_pairs} complete pairs are required; selected {self.pair_count}")
        return (self.scan_points[self.scan_indices].copy(),
                self.camera_points[self.camera_indices].copy(),
                self.scan_indices.copy(), self.camera_indices.copy())


def nearest_depth_pixel(depth, x, y, radius=4):
    """Find a visible point near the click; normalized far-plane depth is 1."""
    depth = np.asarray(depth)
    if depth.ndim != 2 or radius < 0:
        raise ValueError("Expected 2-D depth and a nonnegative click radius")
    height, width = depth.shape
    if not (0 <= x < width and 0 <= y < height):
        return None
    x, y = int(x), int(y)
    x0, x1 = max(0, x-radius), min(width, x+radius+1)
    y0, y1 = max(0, y-radius), min(height, y+radius+1)
    region = depth[y0:y1, x0:x1]
    v, u = np.nonzero(np.isfinite(region) & (region >= 0) & (region < 1))
    if not len(u):
        return None
    u += x0; v += y0
    distance = (u-x)**2+(v-y)**2
    index = int(np.argmin(distance))
    if distance[index] > radius**2:
        return None
    return int(u[index]), int(v[index]), float(depth[v[index], u[index]])


def project_label_pixel(point, view, projection, viewport):
    """Project a point into a viewport INCLUDING its top-left window offset."""
    x, y, width, height = viewport
    if width <= 0 or height <= 0:
        return None
    clip = np.asarray(projection) @ np.asarray(view) @ np.r_[point, 1.]
    if not np.isfinite(clip).all() or clip[3] <= 0:
        return None
    ndc = clip[:3]/clip[3]
    if np.any(np.abs(ndc) > 1):
        return None
    return int(round(x+(ndc[0]+1)*width/2)), int(round(y+(1-ndc[1])*height/2))
