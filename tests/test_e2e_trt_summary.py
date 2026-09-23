"""Exercise the launcher with a fake trtexec; no GPU dependencies."""
import importlib.util
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('trt_summary', ROOT / 'scripts/lib/trt_build_summary.py')
SUMMARY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SUMMARY)
LOG = '''[I] === Performance summary ===
[I] Latency: mean = 2 ms, median = 1.9 ms, percentile(95%) = 3 ms, percentile(99%) = 4 ms
[I] GPU Compute Time: mean = 1 ms, median = 0.9 ms, percentile(95%) = 1.2 ms, percentile(99%) = 1.4 ms
[I] Throughput: 400 qps
[W] GPU compute time is unstable
'''


class TensorRTSummaryTests(unittest.TestCase):
    def test_metrics_and_warnings(self):
        text = SUMMARY.summarize(LOG)
        self.assertIn('GPU計算: 平均 1 / 中央値 0.9 / P95 1.2 / P99 1.4 ms', text)
        self.assertIn('転送込み: 平均 2', text)
        self.assertIn('処理数: 400', text)
        self.assertIn('注意: GPU compute time is unstable', text)
        with self.assertRaises(ValueError):
            SUMMARY.summarize('build failed')

    def test_single_and_split_engines_and_failure(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            model = root / 'model with spaces'
            model.mkdir()
            (model / 'model.onnx').touch()
            stub = root / 'trtexec'
            stub.write_text('#!/usr/bin/env bash\nset -eu\n'
                            'if [[ "${FAIL_TRT:-0}" == 1 ]]; then exit 7; fi\n'
                            'for arg in "$@"; do\n'
                            'case "$arg" in --saveEngine=*) touch "${arg#*=}";; esac\n'
                            'done\ncat <<\'LOG\'\n' + LOG + 'LOG\n')
            stub.chmod(0o755)
            env = {**os.environ, 'TRTEXEC': str(stub), 'JETPILOT_PROJECT_ROOT': str(ROOT)}
            command = ['bash', str(ROOT / 'scripts/e2e_trt.sh'), str(model)]
            single = subprocess.run(command, env=env, capture_output=True, text=True)
            self.assertEqual(single.returncode, 0, single.stderr)
            self.assertTrue((model / 'model.plan').exists())
            self.assertIn('対象: build_engine.log', single.stdout)
            self.assertIn('GPU計算: 平均 1', single.stdout)
            for stem in ('rgb_encoder', 'event_updater'):
                (model / (stem + '.onnx')).touch()
            split = subprocess.run(command, env=env, capture_output=True, text=True)
            self.assertEqual(split.returncode, 0, split.stderr)
            for stem in ('rgb_encoder', 'event_updater'):
                self.assertTrue((model / (stem + '.plan')).exists())
                self.assertIn('対象: build_' + stem + '.log', split.stdout)
            failed = subprocess.run(command, env={**env, 'FAIL_TRT': '1'}, capture_output=True, text=True)
            self.assertNotEqual(failed.returncode, 0)
            self.assertNotIn('=== TensorRT 推論時間のまとめ ===', failed.stdout)


if __name__ == '__main__':
    unittest.main()
