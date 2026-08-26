from __future__ import annotations

import json
import subprocess
import tempfile
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEPLOY_SCRIPT = PACKAGE_ROOT / "scripts/deploy_model.sh"


class DeployModelTests(unittest.TestCase):
    def test_installs_canonical_names_and_invalidates_stale_engine(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            export = root / "export"
            export.mkdir()
            (export / "model.onnx").write_bytes(b"new-onnx")
            (export / "metadata.json").write_text(
                json.dumps({"classes": ["vehicle", "barrier"]}), encoding="utf-8"
            )
            target = root / "models/test-model"
            target.mkdir(parents=True)
            (target / "model.plan").write_bytes(b"stale-engine")

            completed = subprocess.run(
                [
                    str(DEPLOY_SCRIPT),
                    str(export / "model.onnx"),
                    "--model-root",
                    str(root / "models"),
                    "--name",
                    "test-model",
                ],
                text=True,
                capture_output=True,
                check=False,
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertEqual((target / "model.onnx").read_bytes(), b"new-onnx")
            self.assertTrue((target / "metadata.json").is_file())
            self.assertTrue((target / "model.onnx.sha256").is_file())
            self.assertFalse((target / "model.plan").exists())

    def test_remote_deploy_rejects_shell_metacharacters_before_ssh(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            model = root / "model.onnx"
            model.write_bytes(b"onnx")

            completed = subprocess.run(
                [
                    str(DEPLOY_SCRIPT),
                    str(model),
                    "--user",
                    "tamiya",
                    "--host",
                    "10.42.0.1",
                    "--remote-root",
                    "/tmp/yolo';touch-pwned;#",
                    "--yes",
                ],
                text=True,
                capture_output=True,
                check=False,
            )

            self.assertNotEqual(completed.returncode, 0)
            self.assertIn("remote root", completed.stderr)


if __name__ == "__main__":
    unittest.main()
