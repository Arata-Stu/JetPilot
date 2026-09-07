"""Shared validation for map-build model selections."""
from pathlib import Path
import json
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


def scan_models(config):
    """Discover prepared engines without importing CUDA or following arbitrary trees."""
    roots = [Path(config.ros2_ws) / 'isaac_ros_assets/models']
    if getattr(config, 'repo_root', None):
        roots.append(Path(config.repo_root) / 'tools/aliked_workspace/artifacts')
    found = {}
    for root in roots:
        if not root.is_dir():
            continue
        # Asset profiles live directly below models; lab runs use runtime_models.
        for child in sorted(root.iterdir()):
            for candidate in (child, child / 'runtime_models'):
                if candidate.name == 'source_models':
                    continue
                try:
                    folder = resolve_model(config, candidate)
                    engines = folder / 'aliked_lightglue'
                    aliked = sorted(engines.glob('aliked_*.engine'))
                    if not aliked or str(folder) in found:
                        continue
                    matcher = list(engines.glob('lightglue_aliked_*.engine'))
                    width = height = None
                    for record in (folder / 'manifest.json', folder.parent / 'manifest.json'):
                        if record.parent != folder and folder.name != 'runtime_models':
                            continue
                        # Read only bounded metadata within the same allowed roots.
                        if not any(record.resolve().is_relative_to(r.resolve()) for r in roots):
                            continue
                        if not record.is_file() or record.stat().st_size > 1024 * 1024:
                            continue
                        try:
                            data = json.loads(record.read_text())
                            width, height = input_size({'vgl_image_width': data['width'],
                                                       'vgl_image_height': data['height']})
                            break
                        except (ValueError, KeyError, TypeError):
                            continue
                    name = folder.parent.name if folder.name == 'runtime_models' else folder.name
                    found[str(folder)] = dict(path=str(folder), name=name, width=width, height=height,
                                              ready=len(aliked) == 1 and bool(matcher),
                                              size_source='manifest' if width else None)
                except (ValueError, OSError):
                    continue
    return sorted(found.values(), key=lambda item: (not item['ready'], item['name'], item['path']))
