"""Use the exact same dependency-free graph/safety code as the runtime."""
import importlib.util
from pathlib import Path
import sys


def _load(name):
    root = Path(__file__).resolve().parents[4]
    path = root / 'ros2_ws/src/map/jetpilot_hdmap_publisher/jetpilot_hdmap_publisher' / (name + '.py')
    spec = importlib.util.spec_from_file_location('_console_' + name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


network = _load('lane_network')
geometry = _load('drivable_guard')
