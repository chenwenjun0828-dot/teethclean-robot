"""Shared configuration persistence and comment preservation."""
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

from pipeline.oral import (load_jaw_settings as load_jaw, save_jaw_settings as save_jaw,
                           load_mouth_settings as load_mouth, save_mouth_settings as save_mouth,
                           CONFIG_PATH, load_section)


class OralSettingsTests(unittest.TestCase):
    def test_saving_each_preview_group_preserves_the_other_and_comments(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'oral_settings.py'
            source = CONFIG_PATH.read_text(encoding='utf-8')
            source = source.replace("'width_mm': 70.0,", "'width_mm': 70.0,  # 保留宽度注释")
            path.write_text(source, encoding='utf-8')
            original_jaw = load_jaw(path)
            mouth = replace(load_mouth(path), width_mm=62., height_mm=48., distance_mm=None)
            save_mouth(mouth, path)
            self.assertEqual(load_mouth(path), mouth)
            self.assertEqual(load_jaw(path), original_jaw)
            jaw = replace(original_jaw, opening_deg=7.)
            save_jaw(jaw, path)
            self.assertEqual(load_jaw(path), jaw)
            self.assertEqual(load_mouth(path), mouth)
            saved = path.read_text(encoding='utf-8')
            for line in source.splitlines():
                if line.startswith('#'):
                    self.assertIn(line, saved)
            self.assertIn('# 保留宽度注释', saved)

    def test_manual_edit_is_read_again_without_import_cache(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'oral_settings.py'
            source = CONFIG_PATH.read_text(encoding='utf-8')
            path.write_text(source, encoding='utf-8')
            original = load_jaw(path).opening_deg
            path.write_text(source.replace(f"'opening_deg': {original!r}", "'opening_deg': 9.0"),
                            encoding='utf-8')
            self.assertEqual(load_jaw(path).opening_deg, 9.)
            # A missing optional parameter is restored when explicitly saved.
            path.write_text("MOUTH = {'width_mm': 60.0}\nJAW = {'opening_deg': 5.0}\n",
                            encoding='utf-8')
            save_mouth(load_mouth(path), path)
            self.assertEqual(load_mouth(path).width_mm, 60.)
            self.assertEqual(load_jaw(path).opening_deg, 5.)

    def test_literal_configuration_does_not_execute_expressions(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'oral_settings.py'
            path.write_text("MOUTH = {'width_mm': 60 + 10}\nJAW = {}\n", encoding='utf-8')
            with self.assertRaises(ValueError):
                load_section('MOUTH', path)


if __name__ == '__main__':
    unittest.main()
