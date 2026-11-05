"""Synchronized RGB-D snapshots with explicit selection and provenance."""
from __future__ import annotations
import hashlib
import importlib.metadata
import json
from pathlib import Path
import uuid
import mujoco
import numpy as np
import open3d as o3d
from PIL import Image
from .frames import intrinsics_from_fovy, depth_to_points_mm, scan_to_camera_true_mm, camera_world_transform_m

PROJECT_ROOT = Path(__file__).resolve().parents[1]
JAWS = ("mandibular", "maxillary")


def jaw_names(jaw):
    if jaw == "both":
        return JAWS
    if jaw not in JAWS:
        raise ValueError("Unknown jaw")
    return (jaw,)


def file_hash(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024*1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding="utf-8")


def load_config(path=None):
    path = Path(path) if path else PROJECT_ROOT/"config/global_camera_registration.json"
    if not path.is_absolute():
        path = PROJECT_ROOT/path
    config = json.loads(path.read_text(encoding="utf-8-sig"))
    for key in ("model","mandibular_mesh","maxillary_mesh","mandibular_line","maxillary_line"):
        asset = Path(config[key])
        config[key] = str(asset if asset.is_absolute() else PROJECT_ROOT/asset)
        if not Path(config[key]).is_file():
            raise FileNotFoundError(config[key])
    config["config_path"] = str(path.resolve())
    return config


def ensure_empty_directory(path):
    path=Path(path).resolve()
    if path.exists() and (not path.is_dir() or any(path.iterdir())):
        raise FileExistsError(f"Output directory is not empty: {path}")
    path.mkdir(parents=True,exist_ok=True)
    return path


def capture_rgbd(model,data,camera_name,width,height):
    camera_id=model.camera(camera_name).id
    with mujoco.Renderer(model,height=height,width=width) as renderer:
        renderer.update_scene(data,camera=camera_name)
        rgb=renderer.render().copy()
        renderer.enable_depth_rendering()
        depth=renderer.render().copy()
        renderer.enable_segmentation_rendering()
        segmentation=renderer.render().copy()
    return {
        "rgb":rgb,"depth_m":depth,"segmentation":segmentation,
        "intrinsics":intrinsics_from_fovy(width,height,float(model.cam_fovy[camera_id])),
        "camera_name":camera_name,
    }


def valid_depth_mask(depth_m,minimum_mm,maximum_mm):
    if not 0 < minimum_mm < maximum_mm:
        raise ValueError("Invalid depth range")
    return np.isfinite(depth_m) & (depth_m>=minimum_mm/1000) & (depth_m<=maximum_mm/1000)


def selection_mask(frame,model,jaw,config):
    depth=frame["depth_m"]
    mask=valid_depth_mask(depth,config["min_depth_mm"],config["max_depth_mm"])
    if config["mask_mode"] == "ideal_segmentation":
        ids=[model.geom("dental_"+name+"_visual").id for name in jaw_names(jaw)]
        seg=frame["segmentation"]
        mask &= np.isin(seg[:,:,0],ids) & (seg[:,:,1]==int(mujoco.mjtObj.mjOBJ_GEOM))
    elif config["mask_mode"] == "depth_roi":
        x0,y0,x1,y1=map(int,config["roi"])
        height,width=depth.shape
        if not 0<=x0<x1<=width or not 0<=y0<y1<=height:
            raise ValueError("Invalid or empty image ROI")
        roi=np.zeros(depth.shape,bool);roi[y0:y1,x0:x1]=True
        mask &= roi
    else:
        raise ValueError("Unknown mask mode")
    return mask


def make_cloud(points,colors=None):
    cloud=o3d.geometry.PointCloud()
    cloud.points=o3d.utility.Vector3dVector(np.asarray(points,float))
    if colors is not None:
        cloud.colors=o3d.utility.Vector3dVector(np.asarray(colors,float))
    return cloud


def save_cloud(path,cloud):
    if not o3d.io.write_point_cloud(str(path),cloud,write_ascii=False):
        raise OSError(f"Could not write point cloud: {path}")


def save_capture(out,frame,model,data,jaw,config):
    mask=selection_mask(frame,model,jaw,config)
    points,colors=depth_to_points_mm(frame["depth_m"],frame["rgb"],frame["intrinsics"],mask)
    if not len(points):
        raise ValueError("No valid dental points; check camera pose, ROI and depth range")
    shared=np.allclose(data.body("dental_mandibular").xpos,data.body("dental_maxillary").xpos,atol=1e-8) and np.allclose(data.body("dental_mandibular").xmat,data.body("dental_maxillary").xmat,atol=1e-8)
    if jaw=="both" and not shared:
        raise ValueError("Both jaws must share the scan frame; register them separately")
    out=ensure_empty_directory(out)
    Image.fromarray(frame["rgb"]).save(out/"rgb.png")
    np.save(out/"depth_m.npy",frame["depth_m"])
    np.save(out/"segmentation.npy",frame["segmentation"])
    np.save(out/"valid_mask.npy",mask)
    preview=np.zeros(mask.shape,np.uint8)
    preview[mask]=np.clip(
        255*(frame["depth_m"][mask]*1000-config["min_depth_mm"])/
        (config["max_depth_mm"]-config["min_depth_mm"]),0,255,
    ).astype(np.uint8)
    Image.fromarray(preview).save(out/"depth_preview.png")
    save_cloud(out/"camera_mm.ply",make_cloud(points,colors))
    evaluation=out/"evaluation";evaluation.mkdir()
    for name in jaw_names(jaw):
        np.save(evaluation/f"T_camera_scan_true_{name}.npy",
                scan_to_camera_true_mm(data,name,frame["camera_name"]))
    metadata={
        "capture_id":uuid.uuid4().hex,"jaw":jaw,"units":"mm",
        "frame":"camera_cv","depth_unit":"m","camera":frame["camera_name"],
        "intrinsics":frame["intrinsics"],"resolution":[frame["rgb"].shape[1],frame["rgb"].shape[0]],
        "mask_mode":config["mask_mode"],"roi":config["roi"],
        "min_depth_mm":config["min_depth_mm"],"max_depth_mm":config["max_depth_mm"],
        "valid_points":len(points),"simulation_time":float(data.time),"qpos":data.qpos.tolist(),
        "scan_frames_shared":bool(shared),
        "T_world_camera_cv_m":camera_world_transform_m(data,frame["camera_name"]).tolist(),
        "config":config,
        "source_hashes":{name:file_hash(config[name+"_mesh"]) for name in jaw_names(jaw)},
        "reference_line_hashes":{name:file_hash(config[name+"_line"]) for name in jaw_names(jaw)},
        "versions":{package:importlib.metadata.version(package) for package in ("mujoco","numpy","open3d","opencv-python","Pillow")},
        "file_hashes":{name:file_hash(out/name) for name in ("rgb.png","depth_m.npy","segmentation.npy","valid_mask.npy","camera_mm.ply")},
    }
    write_json(out/"capture_metadata.json",metadata)
    return metadata


def load_capture(path):
    path=Path(path).resolve()
    metadata=json.loads((path/"capture_metadata.json").read_text(encoding="utf-8"))
    for name,digest in metadata["file_hashes"].items():
        if file_hash(path/name)!=digest:
            raise ValueError(f"Capture file integrity check failed: {name}")
    return {
        "rgb":np.asarray(Image.open(path/"rgb.png").convert("RGB")),
        "depth_m":np.load(path/"depth_m.npy",allow_pickle=False),
        "segmentation":np.load(path/"segmentation.npy",allow_pickle=False),
        "valid_mask":np.load(path/"valid_mask.npy",allow_pickle=False),
        "intrinsics":metadata["intrinsics"],"metadata":metadata,"path":path,
    }
