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
