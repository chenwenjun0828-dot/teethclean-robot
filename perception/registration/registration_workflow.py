"""RGB-D to oral-scan registration workflow for the active pipeline."""
from __future__ import annotations
import os
os.environ.setdefault("OMP_NUM_THREADS", "4")
import argparse
import json
from pathlib import Path
from time import perf_counter
import numpy as np
import open3d as o3d
from ..frames import validate_transform
from ..global_camera import (
    load_config, load_capture, jaw_names, file_hash, ensure_empty_directory,
    make_cloud, save_cloud, write_json,
)
from .algorithms import (
    load_scan_cloud, load_line_csv, pick_landmark_pairs,
    coarse_from_landmarks, register_clouds,
)
from .registration_math import transform_points


def resolve_registration_method(config, method=None):
    method = config.get("registration_method", "manual") if method is None else method
    if method not in ("manual", "automatic"):
        raise ValueError("Registration method must be manual or automatic")
    return method


def save_registration_transforms(out, initial, estimate):
    """Save both stages in oral_scan_mm -> camera_cv_mm direction."""
    for name, matrix in (("initial", initial), ("estimated", estimate)):
        matrix = validate_transform(matrix)
        np.save(Path(out) / f"T_camera_scan_{name}.npy", matrix)
        np.savetxt(Path(out) / f"T_camera_scan_{name}.txt", matrix, fmt="%.10f",
                   header="oral_scan_mm -> camera_cv_mm")

def checked_inputs(capture_path,config,jaw):
    frame=load_capture(capture_path)
    metadata=frame["metadata"]
    real = metadata.get("data_source") == "realsense"
    if real != (config.get("data_source") == "realsense"):
        raise ValueError("Capture and configuration data sources disagree")
    if real and config.get("T_base_camera_m") is not None:
        from ..calibration import load_base_calibration
        provenance = config.get("base_calibration", {})
        if not provenance.get("path"):
            raise ValueError("Base output requires a calibration file with camera identity")
        matrix, actual = load_base_calibration(provenance["path"], metadata)
        if (actual["sha256"] != provenance.get("sha256")
                or not np.allclose(matrix, validate_transform(config["T_base_camera_m"]), atol=1e-10, rtol=0)):
            raise ValueError("Base calibration changed since this registration was configured")
    if metadata["jaw"]!=jaw:
        raise ValueError("Capture jaw and requested jaw disagree")
    if jaw=="both" and not metadata["scan_frames_shared"]:
        raise ValueError("Moving jaws require separate registrations")
    for name in jaw_names(jaw):
        expected = metadata.get("source_hashes", {}).get(name)
        if (not real or expected is not None) and file_hash(config[name+"_mesh"])!=expected:
            raise ValueError("Scan model changed since capture")
        expected = metadata.get("reference_line_hashes", {}).get(name)
        if config.get(name+"_line") and (not real or expected is not None) and file_hash(config[name+"_line"])!=expected:
            raise ValueError("Reference line changed since capture")
    return frame


def save_overlay_png(path,camera,scan,lines):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(15,5),dpi=150)
    camera_points=np.asarray(camera.points)
    scan_points=np.asarray(scan.points)
    for ax,(first,second) in zip(axes,((0,1),(0,2),(1,2))):
        step=max(1,len(camera_points)//12000)
        color=np.asarray(camera.colors)[::step] if camera.has_colors() else "#707070"
        ax.scatter(camera_points[::step,first],camera_points[::step,second],c=color,s=.3)
        step=max(1,len(scan_points)//12000)
        ax.scatter(scan_points[::step,first],scan_points[::step,second],c="#48b85e",s=.25,alpha=.45)
        for name,points in lines.items():
            ax.scatter(points[:,first],points[:,second],s=.7,label=name)
        ax.set_xlabel("XYZ"[first]+" [mm]");ax.set_ylabel("XYZ"[second]+" [mm]")
        ax.set_aspect("equal")
        if lines:
            ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(path);plt.close(fig)


def run_register(args):
    config=load_config(args.config)
    method=resolve_registration_method(config,getattr(args,"registration_method",None))
    if method=="automatic" and args.landmarks is not None:
        raise ValueError("Automatic registration does not accept manual landmarks")
    frame=checked_inputs(args.capture,config,args.jaw)
    metadata=frame["metadata"]
    real = metadata.get("data_source") == "realsense"
    source_hashes = {name: file_hash(config[name+"_mesh"]) for name in jaw_names(args.jaw)}
    line_hashes = {name: file_hash(config[name+"_line"]) for name in jaw_names(args.jaw)
                   if config.get(name+"_line")}
    # Refuse an existing result before registration or interactive point picking.
    out=ensure_empty_directory(args.out)
    scan=load_scan_cloud([Path(config[name+"_mesh"]) for name in jaw_names(args.jaw)],
                         allow_point_cloud=real)
    camera=o3d.io.read_point_cloud(str(frame["path"]/"camera_mm.ply"))
    registration_started = perf_counter()
    pairs=None
    if method=="automatic":
        from .automatic import AutomaticRegistrationConfig, register_clouds_automatic
        options=AutomaticRegistrationConfig.from_mapping(config.get("automatic_registration",{}))
        estimate,metrics=register_clouds_automatic(scan,camera,options)
        initial=validate_transform(np.asarray(metrics["T_camera_scan_initial"],float))
        landmark_metrics={"landmark_origin":"not_used","landmark_pairs":0,"landmark_inliers":0}
    else:
        if args.landmarks:
            pairs=json.loads(Path(args.landmarks).read_text(encoding="utf-8-sig"))
            if pairs.get("capture_id")!=metadata["capture_id"] or pairs.get("source_hashes")!=source_hashes:
                raise ValueError("Landmarks belong to a different capture or scan")
            source=np.asarray(pairs["source_scan_mm"],float)
            target=np.asarray(pairs["target_camera_mm"],float)
        else:
            source,target,source_indices,target_indices=pick_landmark_pairs(scan,camera,config["minimum_landmarks"])
            pairs={
                "capture_id":metadata["capture_id"],"source_hashes":source_hashes,
                "origin":"manual","source_indices":source_indices,"target_indices":target_indices,
                "source_scan_mm":source.tolist(),"target_camera_mm":target.tolist(),
            }
        initial,inliers,residuals=coarse_from_landmarks(
            source,target,config["minimum_landmarks"],config["landmark_reject_mm"])
        estimate,metrics=register_clouds(
            scan,camera,initial,config["voxel_mm"],config["coarse_distance_mm"],
            config["fine_distance_mm"],config["icp_iterations"],config["icp_direction"],
            config["min_registration_points"])
        pairs=dict(pairs,inliers=inliers.tolist(),initial_residuals_mm=residuals.tolist())
        landmark_metrics={"landmark_origin":pairs.get("origin","unknown"),
                          "landmark_pairs":len(source),"landmark_inliers":int(inliers.sum())}
    metrics["registration_wall_time_s"] = perf_counter() - registration_started
    save_registration_transforms(out,initial,estimate)
    registered=o3d.geometry.PointCloud(scan);registered.transform(estimate)
    save_cloud(out/"scan_registered_camera_mm.ply",registered)
    camera_world=(validate_transform(metadata["T_world_camera_cv_m"]) if not real else None)
    camera_base=(validate_transform(config["T_base_camera_m"]) if real and config.get("T_base_camera_m") is not None else None)
    if camera_base is not None:
        estimate_m = estimate.copy()
        estimate_m[:3, 3] /= 1000
        np.save(out/"T_base_camera_m.npy", camera_base)
        np.save(out/"T_base_scan_m.npy", camera_base @ estimate_m)
    lines={}
    for name in jaw_names(args.jaw):
        if not config.get(name+"_line"):
            continue
        points=transform_points(load_line_csv(config[name+"_line"]),estimate)
        lines[name]=points
        np.savetxt(out/f"{name}_line_camera_mm.csv",np.column_stack((np.arange(len(points)),points)),
            delimiter=",",fmt=["%d","%.8f","%.8f","%.8f"],header="point_index,x_mm,y_mm,z_mm",comments="")
        for label, external in (("world", camera_world), ("base", camera_base)):
            if external is not None:
                world=points/1000@external[:3,:3].T+external[:3,3]
                np.savetxt(out/f"{name}_line_{label}_m.csv",np.column_stack((np.arange(len(points)),world)),
                    delimiter=",",fmt=["%d","%.10f","%.10f","%.10f"],header="point_index,x_m,y_m,z_m",comments="")
        color=[1.,.1,.1] if name=="mandibular" else [1.,.7,0.]
        save_cloud(out/f"{name}_line_camera_mm.ply",make_cloud(points,np.tile(color,(len(points),1))))
    if pairs is not None:
        write_json(out/"landmark_correspondences.json",pairs)
    report={
        "capture_dir":str(frame["path"]),"capture_id":metadata["capture_id"],"jaw":args.jaw,
        "acquisition_id": metadata.get("acquisition_id", metadata["capture_id"]),
        "transform_name":"T_camera_scan","source_frame":"oral_scan","target_frame":"camera_cv",
        "units":"mm","mask_mode":metadata["mask_mode"],"roi":metadata["roi"],
        "source_hashes":source_hashes,"reference_line_hashes":line_hashes,
        "registration_method":method,**landmark_metrics,"valid_points":len(camera.points),
        "config":config,**metrics,
        "data_source": metadata.get("data_source", "simulation"),
        "camera_serial": metadata.get("camera_serial"),
        "optical_frame": metadata.get("optical_frame", "camera_cv"),
        "base_coordinates_available": camera_base is not None,
        "base_calibration": config.get("base_calibration"),
        "artifact_hashes": {name: file_hash(out/name) for name in (
            "T_camera_scan_initial.npy", "T_camera_scan_estimated.npy", "landmark_correspondences.json")
            if (out/name).exists()},
        "simulation_only":not real,"robot_execution_allowed":False,
    }
    write_json(out/"registration_report.json",report)
    save_overlay_png(out/"registration_overlay.png",camera,registered,lines)
    print(f"Saved registration to {out}; ICP RMSE={metrics['icp_fine_rmse_mm']:.4f} mm")
    if not args.no_view:
        geometries=[camera,registered.paint_uniform_color([.2,.85,.3])]
        for name,points in lines.items():
            geometries.append(make_cloud(points,np.tile([1.,.1,.1],(len(points),1))))
        o3d.visualization.draw_geometries(geometries,window_name="Global Camera Registration",width=1280,height=800)
    return 0


def register_capture(capture_dir, config_path, jaw, output_dir, landmarks=None,
                     show_view=False, registration_method=None):
    """Register a saved capture; method overrides config (manual by default).

    Automatic mode uses FPFH/RANSAC/ICP and never asks for landmark picking.
    All saved transforms remain oral-scan millimetres -> camera millimetres.
    """
    args = argparse.Namespace(capture=str(capture_dir),
                              config=str(config_path) if config_path is not None else None,
                              jaw=jaw, out=str(output_dir), landmarks=landmarks,
                              no_view=not show_view, registration_method=registration_method)
    run_register(args)
    return Path(output_dir).resolve()

