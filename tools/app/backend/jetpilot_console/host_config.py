"""Adapter to the project-wide settings shared with shell entry points."""
import importlib.util
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[4]
_spec = importlib.util.spec_from_file_location('jetpilot_project_config', _PROJECT_ROOT / 'scripts/lib/project_config.py')
_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_module)


def host_environment(root=None):
    return _module.load_environment(Path(root) if root else _PROJECT_ROOT)
