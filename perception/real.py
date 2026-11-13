"""Capture a fixed D405, replay registration and evaluate saved results."""
from __future__ import annotations

import argparse
from copy import deepcopy
from pathlib import Path
import sys
from time import perf_counter

import numpy as np
import open3d as o3d

from .acquisition.capture import RGBDFrame, depth_roi_mask, save_real_capture
from .calibration import calibrate_eye_to_hand, load_base_calibration
from .global_camera import PROJECT_ROOT, ensure_empty_directory, load_capture, write_json
from .registration import load_scan_cloud, pick_landmark_pairs, register_capture
from .registration.evaluation import (evaluate_checkpoints, evaluate_registration, read_json,
                                      repeatability_report, load_registration_result)

DEFAULT_CONFIG = PROJECT_ROOT / "config/real_camera_registration.json"


def load_real_config(path=None):
    config = read_json(path or DEFAULT_CONFIG)
    if config.get("data_source") != "realsense":
        raise ValueError("Use a real-camera configuration with data_source=realsense")
    return config


def register_saved_capture(capture_dir, scan, output_dir, *, config=None, jaw=None,
                           registration_method=None, landmarks=None, line=None,
                           calibration=None, checkpoints=None, show_view=False):
    """Replay a capture without importing the camera SDK or creating a simulator."""
    capture_dir, scan = Path(capture_dir).resolve(), Path(scan).resolve()
    frame = load_capture(capture_dir)
    metadata = frame["metadata"]
    if metadata.get("data_source") != "realsense":
        raise ValueError("This entry point requires a saved real-camera capture")
    jaw = metadata["jaw"] if jaw is None else jaw
    if jaw not in ("mandibular", "maxillary") or metadata["jaw"] != jaw:
        raise ValueError("Capture jaw and requested jaw disagree")
    if not scan.is_file():
        raise FileNotFoundError(scan)
    options = deepcopy(config if config is not None else load_real_config())
    if options.get("data_source") != "realsense":
        raise ValueError("Registration configuration must use data_source=realsense")
    # Base outputs require a checked calibration file on every run.
    for key in ("T_base_camera_m", "base_calibration", "mandibular_line", "maxillary_line"):
        options.pop(key, None)
    options[jaw + "_mesh"] = str(scan)
    if line is not None:
        line = Path(line).resolve()
        if not line.is_file():
            raise FileNotFoundError(line)
        options[jaw + "_line"] = str(line)
    if registration_method is not None:
        options["registration_method"] = registration_method
    if calibration is not None:
        matrix, provenance = load_base_calibration(calibration, metadata)
        options["T_base_camera_m"] = matrix.tolist()
        options["base_calibration"] = provenance
    root = ensure_empty_directory(output_dir)
    started = perf_counter()
    summary = {"schema_version": 1, "capture_dir": str(capture_dir),
               "capture_id": metadata["capture_id"], "scan": str(scan), "jaw": jaw,
               "status": "running", "stage": "registration", "robot_execution_allowed": False}
    try:
        config_path = root / "registration_config.json"
        write_json(config_path, options)
        registration = register_capture(capture_dir, config_path, jaw, root / "registration",
                                        landmarks=landmarks, show_view=show_view,
                                        registration_method=registration_method)
        summary["stage"] = "evaluation"
        validation = evaluate_registration(registration, checkpoints)
        summary.update(status="completed", stage="completed", registration_dir=str(registration),
                       validation_status=validation["quality"]["status"],
                       base_coordinates_available=calibration is not None)
    except Exception as error:
        summary.update(status="failed", error=str(error), error_type=type(error).__name__)
        raise
    finally:
        summary["elapsed_s"] = perf_counter() - started
        write_json(root / "run_report.json", summary)
    return summary


def recrop_saved_capture(capture_dir, output_dir, config):
    """Recompute depth ROI and filtering from saved SDK geometry, without hardware."""
    loaded = load_capture(capture_dir)
    metadata, root = deepcopy(loaded["metadata"]), loaded["path"]
    if metadata.get("data_source") != "realsense" or config.get("data_source") != "realsense":
        raise ValueError("Recropping requires real-camera data and configuration")
    names = ("depth_raw", "points_camera_mm", "point_colors", "color_valid_mask")
    if any(name + ".npy" not in metadata["file_hashes"] for name in names):
        raise ValueError("Recropping requires fingerprinted raw depth and SDK point/color maps")
    raw, points, colors, color_valid = [np.load(root / (name + ".npy"), allow_pickle=False) for name in names]
    metadata["parent_capture_id"] = metadata["capture_id"]
    metadata["parent_capture_dir"] = str(root)
    metadata["acquisition_id"] = metadata.get("acquisition_id", metadata["capture_id"])
    frame = RGBDFrame(loaded["rgb"], raw, metadata["depth_scale_m"], points, colors,
                      color_valid, metadata["intrinsics"], metadata)
    return save_real_capture(output_dir, frame, metadata["jaw"], config)


def _capture(args):
    from .acquisition.realsense import RealSenseCamera

    import cv2

    config = load_real_config(args.config)
    if args.serial:
        config["camera"]["serial"] = args.serial
    if args.count < 1:
        raise ValueError("Capture count must be positive")
    out = ensure_empty_directory(args.out)
    captures = []
    window = "D405 depth-grid selection"
    try:
        with RealSenseCamera(config["camera"]) as source:
            for index in range(args.count):
                frame = source.read()
                if args.select_roi and index == 0:
                    image = np.rint(frame.colors[:, :, ::-1] * 255).astype(np.uint8)
                    x, y, width, height = cv2.selectROI(window, image, showCrosshair=True, fromCenter=False)
                    if width <= 0 or height <= 0:
                        raise RuntimeError("ROI selection cancelled")
                    config["roi"] = [int(x), int(y), int(x + width), int(y + height)]
                    cv2.destroyWindow(window)
                if args.preview:
                    print("Preview: S/Space saves; Q/Esc cancels. ROI uses native depth pixels.")
                    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
                    while True:
                        mask, roi = depth_roi_mask(frame.depth_m, config)
                        image = np.rint(frame.colors[:, :, ::-1] * 255).astype(np.uint8)
                        image[~mask] //= 4
                        cv2.rectangle(image, tuple(roi[:2]), tuple(roi[2:]), (0, 255, 0), 1)
                        cv2.imshow(window, image)
                        key = cv2.waitKey(1) & 0xff
                        if key in (ord("s"), ord("S"), 32):
                            break
                        if key in (ord("q"), 27) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                            raise RuntimeError("Capture cancelled")
                        frame = source.read()
                directory = out if args.count == 1 else out / f"capture_{index + 1:04d}"
                metadata = save_real_capture(directory, frame, args.jaw, config)
                captures.append({"path": str(directory), "capture_id": metadata["capture_id"]})
                print(f"Saved {directory}: {metadata['valid_points']} points; depth scale={frame.depth_scale_m} m/unit")
    finally:
        if args.preview or args.select_roi:
            cv2.destroyAllWindows()
        if args.count > 1:
            write_json(out / "session.json", {"captures": captures, "requested_count": args.count,
                                             "complete": len(captures) == args.count})
    return 0


def _pick_checkpoints(args):
    directory = Path(args.registration).resolve()
    report, transform = load_registration_result(directory)
    capture = load_capture(report["capture_dir"])
    scan = load_scan_cloud([Path(report["config"][report["jaw"] + "_mesh"])], allow_point_cloud=True)
    camera = o3d.io.read_point_cloud(str(capture["path"] / "camera_mm.ply"))
    target = Path(args.out)
    if target.exists():
        raise FileExistsError(target)
    source, observed, source_indices, target_indices = pick_landmark_pairs(scan, camera, 3)
    payload = {"capture_id": report["capture_id"], "source_hashes": report["source_hashes"],
               "source_scan_mm": source.tolist(), "target_camera_mm": observed.tolist(),
               "source_indices": source_indices, "target_indices": target_indices,
               "independent_of_registration": True, "measurement_method": "manual_withheld_point_identification"}
    pairs_path = directory / "landmark_correspondences.json"
    evaluate_checkpoints(transform, payload, report, read_json(pairs_path) if pairs_path.exists() else None)
    write_json(target, payload)
    print(f"Saved withheld checkpoints to {target}; point identification uncertainty is not measured")
    return 0


def build_argparser():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    from .registration.manual_real import add_arguments
    add_arguments(commands.add_parser('manual', help='PLY/sample/D405 -> manual landmarks -> ICP -> optional base coordinates'))
    commands.add_parser("devices", help="List connected RealSense devices")
    capture = commands.add_parser("capture", help="Save D405 snapshots without running registration")
    capture.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    capture.add_argument("--out", type=Path, required=True)
    capture.add_argument("--jaw", choices=("mandibular", "maxillary"), required=True)
    capture.add_argument("--serial")
    capture.add_argument("--count", type=int, default=1)
    capture.add_argument("--preview", action="store_true")
    capture.add_argument("--select-roi", action="store_true")
    recrop = commands.add_parser("recrop", help="Reprocess saved raw geometry with a different ROI or depth range")
    recrop.add_argument("--capture", type=Path, required=True)
    recrop.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    recrop.add_argument("--out", type=Path, required=True)
    replay = commands.add_parser("register", help="Register a saved real capture")
    replay.add_argument("--capture", type=Path, required=True)
    replay.add_argument("--scan", type=Path, required=True, help="Matching oral scan in millimetres")
    replay.add_argument("--out", type=Path, required=True)
    replay.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    replay.add_argument("--jaw", choices=("mandibular", "maxillary"))
    replay.add_argument("--method", choices=("automatic", "manual"))
    for name in ("landmarks", "line", "calibration", "checkpoints"):
        replay.add_argument("--" + name, type=Path)
    replay.add_argument("--view", action="store_true")
    checkpoints = commands.add_parser("checkpoints", help="Pick withheld points after registration")
    checkpoints.add_argument("--registration", type=Path, required=True)
    checkpoints.add_argument("--out", type=Path, required=True)
    evaluate = commands.add_parser("evaluate", help="Evaluate a registration with withheld checkpoints")
    evaluate.add_argument("--registration", type=Path, required=True)
    evaluate.add_argument("--checkpoints", type=Path, required=True)
    evaluate.add_argument("--out", type=Path, required=True)
    repeat = commands.add_parser("repeatability", help="Compare runs of an explicitly static scene")
    repeat.add_argument("--registrations", nargs="+", type=Path, required=True)
    repeat.add_argument("--static-scene", action="store_true")
    repeat.add_argument("--out", type=Path, required=True)
    calibration = commands.add_parser("calibrate", help="Solve eye-to-hand from recorded robot/board poses")
    calibration.add_argument("--samples", type=Path, required=True)
    calibration.add_argument("--validation-samples", type=Path)
    calibration.add_argument("--out", type=Path, required=True)
    return parser


def main(argv=None):
    parser = build_argparser()
    args = parser.parse_args(argv)
    if args.command == 'manual':
        from .registration.manual_real import run_manual
        return run_manual(args, parser)
    try:
        if args.command == "devices":
            from .acquisition.realsense import list_devices
            devices = list_devices()
            for device in devices:
                print(f"{device['name']} serial={device['serial']} firmware={device['firmware']}")
            if not devices:
                print("No RealSense devices connected")
            return 0
        if args.command == "capture":
            return _capture(args)
        if args.command == "recrop":
            metadata = recrop_saved_capture(args.capture, args.out, load_real_config(args.config))
            print(f"Saved {args.out}: {metadata['valid_points']} points; original acquisition preserved")
            return 0
        if args.command == "register":
            result = register_saved_capture(args.capture, args.scan, args.out,
                config=load_real_config(args.config), jaw=args.jaw, registration_method=args.method,
                landmarks=args.landmarks, line=args.line, calibration=args.calibration,
                checkpoints=args.checkpoints, show_view=args.view)
            print(f"Saved {result['registration_dir']}; validation={result['validation_status']}")
            return 2 if result["validation_status"] in ("failed", "not_evaluable") else 0
        if args.command == "checkpoints":
            return _pick_checkpoints(args)
        if args.command == "evaluate":
            result = evaluate_registration(args.registration, args.checkpoints, output_path=args.out)
            print(f"Checkpoint RMS={result['checkpoints']['rms_mm']:.4f} mm; quality={result['quality']['status']}")
            return 2 if result["quality"]["status"] in ("failed", "not_evaluable") else 0
        if Path(args.out).exists():
            raise FileExistsError(args.out)
        if args.command == "repeatability":
            result = repeatability_report(args.registrations, static_scene=args.static_scene)
        else:
            result = calibrate_eye_to_hand(read_json(args.samples),
                read_json(args.validation_samples) if args.validation_samples else None)
        write_json(args.out, result)
        print(f"Saved {args.out}")
        return 0
    except (ValueError, RuntimeError, OSError, KeyError, TypeError) as error:
        print(f"Real-camera {args.command} failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
