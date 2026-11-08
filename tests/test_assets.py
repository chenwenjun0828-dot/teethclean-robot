import json
from pathlib import Path
import tempfile
import unittest

from assets.tools.build_manifest import build_manifest


class AssetManifestTests(unittest.TestCase):
    def test_manifest_is_deterministic_and_contains_file_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "simulation" / "robot").mkdir(parents=True)
            (root / "source_cad").mkdir()
            (root / "perception").mkdir()
            (root / "simulation" / "robot" / "scene.xml").write_text("<mujoco/>", encoding="utf-8")
            manifest = build_manifest(root)
        self.assertEqual(manifest["schema_version"], 1)
        self.assertEqual(manifest["files"][0]["path"], "simulation/robot/scene.xml")
        self.assertEqual(manifest["files"][0]["format"], "xml")
        self.assertEqual(len(manifest["files"][0]["sha256"]), 64)

    def test_checked_in_manifest_has_expected_roots(self):
        payload = json.loads(Path("assets/manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(payload["asset_roots"], ["simulation", "source_cad", "perception"])
