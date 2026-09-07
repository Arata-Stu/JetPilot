import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from jetpilot_console.vgl_models import input_size, resolve_model

class VglModelsTest(unittest.TestCase):
    def test_dimensions(self):
        self.assertEqual(input_size({}), (1920, 1200))
        self.assertEqual(input_size({'vgl_image_width':424, 'vgl_image_height':240}), (424,240))
        for value in (True, 0, -1, 1.5, '424;echo bad', 10000):
            with self.assertRaises(ValueError):
                input_size({'vgl_image_width':value})

    def test_lab_path_and_symlink_escape(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            config = SimpleNamespace(repo_root=repo, ros2_ws=repo / 'ros2_ws')
            target = repo / 'tools/aliked_workspace/artifacts/small/runtime_models'
            target.mkdir(parents=True)
            self.assertEqual(resolve_model(config, target), target.resolve())
            outside = repo / 'outside'
            outside.mkdir()
            escape = target.parent / 'escape'
            escape.symlink_to(outside)
            with self.assertRaises(ValueError):
                resolve_model(config, escape)

class ScanModelsTest(unittest.TestCase):
    def test_scan_finds_assets_and_lab_without_guessing_size_from_name(self):
        from jetpilot_console.vgl_models import scan_models
        import json
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            config = SimpleNamespace(repo_root=root, ros2_ws=root / 'ros2_ws')
            assets = config.ros2_ws / 'isaac_ros_assets/models/visual_global_localization_424x240'
            run = root / 'tools/aliked_workspace/artifacts/small'
            runtime = run / 'runtime_models'
            for folder in (assets, runtime, run / 'source_models'):
                engines = folder / 'aliked_lightglue'
                engines.mkdir(parents=True)
                (engines / 'aliked_test.engine').touch()
                (engines / 'lightglue_aliked_test.engine').touch()
            (run / 'manifest.json').write_text(json.dumps(dict(width=424, height=240)))
            result = scan_models(config)
            self.assertEqual(len(result), 2)
            lab = next(v for v in result if v['name'] == 'small')
            self.assertEqual((lab['width'], lab['height']), (424,240))
            self.assertTrue(lab['ready'])
            asset = next(v for v in result if v['name'].startswith('visual_'))
            self.assertIsNone(asset['width'])

    def test_scan_ignores_external_links_and_bad_metadata(self):
        from jetpilot_console.vgl_models import scan_models
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            config = SimpleNamespace(repo_root=root, ros2_ws=root / 'ros2_ws')
            models = config.ros2_ws / 'isaac_ros_assets/models'
            models.mkdir(parents=True)
            outside = root / 'outside'
            engines = outside / 'aliked_lightglue'
            engines.mkdir(parents=True)
            (engines / 'aliked_test.engine').touch()
            (models / 'escape').symlink_to(outside)
            self.assertEqual(scan_models(config), [])
            inside = models / 'broken'
            (inside / 'aliked_lightglue').mkdir(parents=True)
            (inside / 'aliked_lightglue/aliked_test.engine').touch()
            (inside / 'manifest.json').write_text('{bad json')
            found = scan_models(config)
            self.assertEqual(len(found), 1)
            self.assertFalse(found[0]['ready'])
            self.assertIsNone(found[0]['width'])
