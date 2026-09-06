"""Shared validation for map-build model selections."""
from pathlib import Path
from .security import resolve_under_root


def input_size(payload):
    result = []
    for name, default in (("vgl_image_width", 1920), ("vgl_image_height", 1200)):
        value = payload.get(name, default)
        if isinstance(value, bool) or not str(value).isdigit():
            raise ValueError(f"{name} must be a positive integer")
        value = int(value)
        if not 1 <= value <= 8192:
            raise ValueError(f"{name} must be between 1 and 8192")
        result.append(value)
    if result[0] * result[1] < 2048:
        raise ValueError("ALIKED input must contain at least 2048 pixels")
    return tuple(result)


def resolve_model(config, selected):
    model_root = Path(config.ros2_ws) / "isaac_ros_assets/models"
    selected = selected or model_root / "visual_global_localization"
    roots = [model_root]
    if getattr(config, 'repo_root', None):
        roots.append(Path(config.repo_root) / "tools/aliked_workspace/artifacts")
    for root in roots:
        try:
            return resolve_under_root(selected, root, label="VGL model directory",
                                      require_exists=True, require_directory=True)
        except ValueError:
            continue
    raise ValueError("VGL model directory must exist inside workspace models or ALIKED experiment artifacts")
