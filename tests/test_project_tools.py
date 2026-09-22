"""Workspace maintenance tests; no ROS, NumPy, YAML or network required."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'scripts/lib' / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class HostConfigTests(unittest.TestCase):
    def test_precedence_derived_paths_and_no_shell_execution(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'config').mkdir()
            (root / 'config/host.json').write_text(json.dumps({'JETSON_REMOTE_USER':'base', 'JETSON_WORKSPACE_ROOT':'/srv/base'}))
            (root / 'config/host.local.json').write_text(json.dumps({'JETSON_REMOTE_USER':'local', 'JETSON_WORKSPACE_ROOT':'/srv/robot space'}))
            env = load('project_config').load_environment(root, {'JETSON_REMOTE_USER':'override'})
            self.assertEqual(env['JETSON_REMOTE_USER'], 'override')
            self.assertEqual(env['JETSON_MAP_ROOT'], '/srv/robot space/map')
            self.assertEqual(env['ROS2_WS'], str(root / 'ros2_ws'))
            (root / 'config/host.local.json').write_text('{"PATH":"/evil"}')
            with self.assertRaises(ValueError):
                load('project_config').load_environment(root, {})

    def test_console_reads_same_local_configuration(self):
        sys.path.insert(0, str(ROOT / 'tools/app/backend'))
        from jetpilot_console.host_config import host_environment
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'config').mkdir()
            (root / 'config/host.local.json').write_text('{"JETSON_REMOTE_USER":"test-robot"}')
            self.assertEqual(host_environment(root)['JETSON_REMOTE_USER'], 'test-robot')


class BuildTests(unittest.TestCase):
    def run_build(self, root, *args, **kwargs):
        return subprocess.run([sys.executable, '-S', str(ROOT / 'scripts/lib/build_workspace.py'), '--workspace', str(root), *args], capture_output=True, text=True, **kwargs)

    def test_selected_build_preserves_other_outputs_and_plans_dependencies(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'src/pkg').mkdir(parents=True)
            (root / 'src/pkg/package.xml').write_text('<package><name>pkg</name></package>')
            (root / 'build/other').mkdir(parents=True)
            result = self.run_build(root, '--packages', 'pkg', '--clean', '--jobs', '2', '--dry-run')
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('--packages-up-to pkg', result.stdout)
            self.assertIn('--parallel-workers 1', result.stdout)
            self.assertIn(str(root / 'build/pkg'), result.stdout)
            self.assertNotIn(str(root / 'build/other'), result.stdout)
            self.assertTrue((root / 'build/other').exists())
            explicit = self.run_build(root, '--packages', 'pkg', '--no-deps', '--dry-run')
            self.assertIn('--packages-select pkg', explicit.stdout)
            invalid = self.run_build(root, '--packages', '../other', '--clean')
            self.assertNotEqual(invalid.returncode, 0)
            noninteractive = self.run_build(root, input='2\n')
            self.assertNotEqual(noninteractive.returncode, 0)

    def test_clean_rejects_parent_symlink_outside_workspace(self):
        with tempfile.TemporaryDirectory() as folder:
            base = Path(folder)
            root = base / 'workspace'
            (root / 'src/pkg').mkdir(parents=True)
            (root / 'src/pkg/package.xml').write_text('<package><name>pkg</name></package>')
            outside = base / 'outside'
            (outside / 'pkg').mkdir(parents=True)
            (root / 'build').symlink_to(outside, target_is_directory=True)
            result = self.run_build(root, '--packages', 'pkg', '--clean', '--dry-run')
            self.assertNotEqual(result.returncode, 0)
            self.assertTrue((outside / 'pkg').is_dir())

    def test_ignored_nested_packages_are_not_candidates(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'src/ignored/pkg').mkdir(parents=True)
            (root / 'src/ignored/COLCON_IGNORE').touch()
            (root / 'src/ignored/pkg/package.xml').write_text('<package><name>pkg</name></package>')
            self.assertEqual(load('build_workspace').package_names(root), [])


class RepositoryTests(unittest.TestCase):
    def git(self, path, *args):
        return subprocess.run(['git', '-C', str(path), *args], check=True, capture_output=True, text=True).stdout.strip()

    def run_repos(self, root, *args):
        return subprocess.run([sys.executable, '-S', str(ROOT / 'scripts/lib/repositories.py'), *args, '--root', str(root)], text=True, capture_output=True)

    def test_pull_is_fast_forward_only_and_dirty_checkout_is_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            origin = root / 'origin'
            origin.mkdir()
            self.git(origin, 'init', '-b', 'main')
            self.git(origin, 'config', 'user.email', 'test@example.invalid')
            self.git(origin, 'config', 'user.name', 'Test')
            (origin / 'file').write_text('one')
            self.git(origin, 'add', 'file')
            self.git(origin, 'commit', '-m', 'initial')
            workspace = root / 'workspace'
            workspace.mkdir()
            (workspace / 'packages.repos').write_text(f'repositories:\n  tools/demo:\n    type: git\n    url: {origin}\n    version: main\n')
            result = self.run_repos(workspace, 'import')
            self.assertEqual(result.returncode, 0, result.stderr)
            checkout = workspace / 'tools/demo'
            (origin / 'file').write_text('two')
            self.git(origin, 'commit', '-am', 'next')
            self.assertEqual(self.run_repos(workspace, 'pull').returncode, 0)
            self.assertEqual((checkout / 'file').read_text(), 'two')
            self.assertEqual(self.run_repos(workspace, 'lock').returncode, 0)
            lock = (workspace / 'packages.lock.repos').read_text()
            self.assertIn(self.git(checkout, 'rev-parse', 'HEAD'), lock)
            (checkout / 'file').write_text('uncommitted')
            self.assertNotEqual(self.run_repos(workspace, 'pull').returncode, 0)
            self.assertNotEqual(self.run_repos(workspace, 'lock').returncode, 0)
            self.assertEqual((checkout / 'file').read_text(), 'uncommitted')
            self.assertEqual((workspace / 'packages.lock.repos').read_text(), lock)
            self.git(checkout, 'checkout', '--', 'file')
            self.git(checkout, 'checkout', '--detach')
            before = self.git(checkout, 'rev-parse', 'HEAD')
            self.assertEqual(self.run_repos(workspace, 'pull').returncode, 0)
            self.assertEqual(self.git(checkout, 'rev-parse', 'HEAD'), before)

    def test_manifest_rejects_path_traversal(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'packages.repos'
            path.write_text('repositories:\n  ../outside:\n    type: git\n    url: example\n    version: main\n')
            with self.assertRaises(ValueError):
                load('repositories').read_manifest(path)


if __name__ == '__main__':
    unittest.main()
