import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location('profile', Path(__file__).parents[1] / 'launch/vgl_model_profile.py')
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)

class ProfileTest(unittest.TestCase):
    def setup_map(self, root, identity='wanted'):
        mapdir = root / 'map'
        config = mapdir / 'vgl_runtime_config'
        config.mkdir(parents=True)
        (config / 'keypoint_creation_config.pb.txt').write_text('aliked_detector { opt: 1 opt: 3 opt: 240 opt: 424 }')
        profile = dict(status='complete', width=424, height=240, model_dir='/missing/build/host', onnx_sha256=identity)
        (mapdir / 'vgl_profile.json').write_text(json.dumps(profile))
        return mapdir

    def model(self, root, name):
        folder = root / name / 'runtime_models'
        engines = folder / 'aliked_lightglue'
        engines.mkdir(parents=True)
        (engines / 'aliked_test.engine').write_bytes(b'engine')
        (engines / 'lightglue_aliked_test.engine').touch()
        onnx = folder.parent / 'aliked.onnx'
        onnx.write_bytes(name.encode())
        (folder.parent / 'manifest.json').write_text(json.dumps({'onnx_sha256':m.digest(onnx)}))
        return folder, m.digest(onnx)

    def test_cross_host_selects_same_onnx(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            folder, identity = self.model(root / 'models', 'small')
            self.model(root / 'models', 'wrong')
            mapdir = self.setup_map(root, identity)
            selected = m.resolve_profile(mapdir / 'cuvgl_map', 'auto', 'auto', '/default', [root / 'models'])
            self.assertEqual(selected[0], str(folder.resolve()))
            self.assertEqual(selected[1], str(mapdir / 'vgl_runtime_config'))

    def test_missing_model_never_falls_back(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            mapdir = self.setup_map(root)
            with self.assertRaisesRegex(ValueError, '0 matching'):
                m.resolve_profile(mapdir, 'auto', 'auto', '/default', [])

    def test_explicit_override_and_legacy_map(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.assertEqual(m.resolve_profile(root,'auto','auto','/default',[])[:2],(m.DEFAULT_MODEL,'/default'))
            mapdir = self.setup_map(root)
            self.assertEqual(m.resolve_profile(mapdir,'/override','auto','/default',[])[0],'/override')

    def test_wrong_config_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            mapdir = self.setup_map(Path(tmp))
            (mapdir / 'vgl_runtime_config/keypoint_creation_config.pb.txt').write_text('opt: 1 opt: 3 opt: 1200 opt: 1920')
            with self.assertRaisesRegex(ValueError, 'input size'):
                m.resolve_profile(mapdir,'auto','auto','/default',[])

    def test_changed_onnx_is_not_selected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            folder, identity=self.model(root / 'models', 'small')
            mapdir=self.setup_map(root, identity)
            (folder.parent / 'aliked.onnx').write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError, '0 matching'):
                m.resolve_profile(mapdir,'auto','auto','/default',[root/'models'])

    def test_existing_profile_uses_recorded_path_and_local_inspection(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            folder, _ = self.model(root / 'models', 'small')
            mapdir = self.setup_map(root, None)
            record = m.read_json(mapdir / 'vgl_profile.json')
            record['model_dir'] = str(folder)
            (mapdir / 'vgl_profile.json').write_text(json.dumps(record))
            engine = folder / 'aliked_lightglue/aliked_test.engine'
            inspection = dict(file=str(engine), sha256=m.digest(engine),
                              tensors=[dict(name='image', shape=[1,3,240,424])])
            (folder.parent / 'engine-inspection.json').write_text(json.dumps(inspection))
            self.assertEqual(m.resolve_profile(mapdir,'auto','auto','/default',[])[0],str(folder.resolve()))
            engine.write_bytes(b'replaced')
            with self.assertRaisesRegex(ValueError, '0 matching'):
                m.resolve_profile(mapdir,'auto','auto','/default',[])

    def test_ambiguous_models_require_explicit_selection(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            folder, identity = self.model(root / 'models', 'small')
            import shutil
            shutil.copytree(folder.parent, root / 'models/duplicate')
            mapdir = self.setup_map(root, identity)
            with self.assertRaisesRegex(ValueError, '2 matching'):
                m.resolve_profile(mapdir,'auto','auto','/default',[root/'models'])
