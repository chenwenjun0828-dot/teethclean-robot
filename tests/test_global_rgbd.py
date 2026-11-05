import tempfile
import unittest
from pathlib import Path
import mujoco
import numpy as np
import open3d as o3d
from pipeline import simulation
from perception.global_camera import (
    capture_rgbd, selection_mask, save_capture, load_capture, load_config,
)


class GlobalRGBDTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model, cls.data = simulation.create_simulation(simulation.DENTAL_MODEL_PATH)
        cls.config = load_config()
        cls.frame = capture_rgbd(cls.model,cls.data,"global_dental_camera",1280,720)

    def test_actual_capture_preserves_simulation_and_has_both_jaws(self):
        self.assertEqual(self.frame["rgb"].shape,(720,1280,3))
        self.assertEqual(self.frame["rgb"].dtype,np.uint8)
        self.assertEqual(self.frame["depth_m"].dtype,np.float32)
        for jaw in ("mandibular","maxillary"):
            mask=selection_mask(self.frame,self.model,jaw,self.config)
            self.assertGreater(mask.sum(),5000)
            self.assertTrue((self.frame["depth_m"][mask]>.04).all())
            self.assertTrue((self.frame["depth_m"][mask]<.3).all())
        self.assertEqual(float(self.data.time),0.)

    def test_save_reload_cloud_and_refuse_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            out=Path(directory)/"capture"
            metadata=save_capture(out,self.frame,self.model,self.data,"both",self.config)
            restored=load_capture(out)
            np.testing.assert_array_equal(restored["depth_m"],self.frame["depth_m"])
            cloud=o3d.io.read_point_cloud(str(out/"camera_mm.ply"))
            self.assertEqual(len(cloud.points),metadata["valid_points"])
            np.testing.assert_allclose(
                np.asarray(cloud.points)[:,2],self.frame["depth_m"][restored["valid_mask"]]*1000,
                atol=0.001,
            )
            with self.assertRaises(FileExistsError):
                save_capture(out,self.frame,self.model,self.data,"both",self.config)
            with (out/"depth_m.npy").open("ab") as handle:
                handle.write(b"tampered")
            with self.assertRaises(ValueError):
                load_capture(out)

    def test_depth_roi_does_not_use_labels(self):
        synthetic={"depth_m":np.full((4,4),.2),"segmentation":np.full((4,4,2),-1)}
        config=dict(self.config,mask_mode="depth_roi",roi=[1,1,3,3])
        mask=selection_mask(synthetic,self.model,"both",config)
        self.assertEqual(mask.sum(),4)
        config["roi"]=[4,4,4,4]
        with self.assertRaises(ValueError):
            selection_mask(synthetic,self.model,"both",config)
