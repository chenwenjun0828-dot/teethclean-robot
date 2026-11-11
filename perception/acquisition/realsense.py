"""D405 source returning native depth-optical frames; SDK is loaded on demand."""
from __future__ import annotations

from datetime import datetime, timezone
import importlib
import importlib.metadata
from time import perf_counter

import numpy as np

from .capture import RGBDFrame
from ..frames import validate_transform


def load_sdk():
    try:
        return importlib.import_module("pyrealsense2")
    except ImportError as error:
        raise RuntimeError("Install the camera dependencies: python -m pip install -r requirements-real-camera.txt") from error


def list_devices(sdk=None):
    rs = sdk if sdk is not None else load_sdk()
    return [{"name": device.get_info(rs.camera_info.name),
             "serial": device.get_info(rs.camera_info.serial_number),
             "firmware": device.get_info(rs.camera_info.firmware_version)}
            for device in rs.context().query_devices()]


def _intrinsics(profile):
    value = profile.get_intrinsics()
    return {"fx": value.fx, "fy": value.fy, "cx": value.ppx, "cy": value.ppy,
            "width": value.width, "height": value.height,
            "distortion_model": str(value.model), "distortion_coeffs": list(value.coeffs)}


def _texture_colors(rgb, uv):
    height, width = rgb.shape[:2]
    valid = np.isfinite(uv).all(axis=2) & (uv >= 0).all(axis=2) & (uv < 1).all(axis=2)
    colors = np.full(uv.shape[:2] + (3,), .5, np.float32)
    x = np.minimum((uv[valid, 0] * width + .5).astype(int), width - 1)
    y = np.minimum((uv[valid, 1] * height + .5).astype(int), height - 1)
    colors[valid] = rgb[y, x].astype(np.float32) / 255
    return colors, valid


class RealSenseCamera:
    """Keep one SDK pipeline open; persistence is deliberately outside read()."""

    def __init__(self, options, sdk=None):
        self.options = dict(options)
        for name in ("width", "height", "fps", "timeout_ms"):
            value = self.options[name]
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"Camera {name} must be a positive integer")
        warmup = self.options.get("warmup_frames", 30)
        if isinstance(warmup, bool) or not isinstance(warmup, int) or warmup < 0:
            raise ValueError("warmup_frames must be a nonnegative integer")
        self.rs = sdk
        self.pipeline = None

    def __enter__(self):
        if self.pipeline is not None:
            raise RuntimeError("Camera source is already open")
        rs = self.rs if self.rs is not None else load_sdk()
        devices = [device for device in list_devices(rs) if "D405" in device["name"]]
        serial = self.options.get("serial")
        if serial:
            devices = [device for device in devices if device["serial"] == serial]
        if not devices:
            raise RuntimeError("No matching D405 found; check USB connection, serial and RealSense Viewer")
        if len(devices) != 1:
            raise ValueError("Multiple D405 cameras found; select camera.serial explicitly")
        self.device_info = devices[0]
        config = rs.config()
        config.enable_device(self.device_info["serial"])
        for stream, fmt in ((rs.stream.depth, rs.format.z16), (rs.stream.color, rs.format.rgb8)):
            config.enable_stream(stream, self.options["width"], self.options["height"], fmt, self.options["fps"])
        pipeline = rs.pipeline()
        started = False
        try:
            profile = pipeline.start(config)
            started = True
            self.scale = float(profile.get_device().first_depth_sensor().get_depth_scale())
            if not np.isfinite(self.scale) or self.scale <= 0:
                raise ValueError("D405 returned an invalid depth scale")
            self.depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
            self.color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
            extrinsics = self.depth_profile.get_extrinsics_to(self.color_profile)
            self.T_color_depth_m = np.eye(4)
            self.T_color_depth_m[:3, :3] = np.asarray(extrinsics.rotation).reshape(3, 3, order="F")
            self.T_color_depth_m[:3, 3] = extrinsics.translation
            validate_transform(self.T_color_depth_m)
            self.pc = rs.pointcloud()
            self.pipeline = pipeline
            self.rs = rs
            for _ in range(self.options.get("warmup_frames", 30)):
                self.pipeline.wait_for_frames(self.options["timeout_ms"])
        except BaseException:
            if started:
                pipeline.stop()
            self.pipeline = None
            raise
        return self

    def read(self):
        if self.pipeline is None:
            raise RuntimeError("Camera source is not open")
        started = perf_counter()
        frames = self.pipeline.wait_for_frames(self.options["timeout_ms"])
        depth, color = frames.get_depth_frame(), frames.get_color_frame()
        if not depth or not color:
            raise RuntimeError("D405 frameset is missing depth or color")
        received = datetime.now(timezone.utc).isoformat()
        received_monotonic = perf_counter()
        raw = np.asanyarray(depth.get_data()).copy()
        rgb = np.asanyarray(color.get_data()).copy()
        self.pc.map_to(color)
        points = self.pc.calculate(depth)
        # SDK vertices already account for depth intrinsics and distortion.
        xyz = np.asanyarray(points.get_vertices()).view(np.float32).reshape(raw.shape + (3,)).copy() * 1000
        uv = np.asanyarray(points.get_texture_coordinates()).view(np.float32).reshape(raw.shape + (2,)).copy()
        colors, color_valid = _texture_colors(rgb, uv)
        try:
            sdk_version = importlib.metadata.version("pyrealsense2")
        except importlib.metadata.PackageNotFoundError:
            sdk_version = None
        metadata = {
            "device": self.device_info, "camera_serial": self.device_info["serial"],
            "host_received_at_utc": received,
            "host_received_monotonic_s": received_monotonic,
            "sdk_version": sdk_version,
            "depth_timestamp_ms": float(depth.get_timestamp()),
            "color_timestamp_ms": float(color.get_timestamp()),
            "depth_timestamp_domain": str(depth.get_frame_timestamp_domain()),
            "color_timestamp_domain": str(color.get_frame_timestamp_domain()),
            "depth_frame_number": int(depth.get_frame_number()),
            "color_frame_number": int(color.get_frame_number()),
            "color_intrinsics": _intrinsics(self.color_profile),
            "T_color_depth_m": self.T_color_depth_m.tolist(),
            "camera_options": self.options, "read_elapsed_s": perf_counter() - started,
            "geometry_method": "librealsense_native_depth_pointcloud",
        }
        return RGBDFrame(rgb, raw, self.scale, xyz, colors, color_valid,
                         _intrinsics(self.depth_profile), metadata)

    def close(self):
        pipeline, self.pipeline = self.pipeline, None
        if pipeline is not None:
            pipeline.stop()

    def __exit__(self, *exc):
        self.close()
