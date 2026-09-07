"""Resolve a map's VGL model without importing ROS or CUDA."""
import hashlib
import json
import re
from pathlib import Path

DEFAULT_MODEL = '/workspaces/ros2_ws/isaac_ros_assets/models/visual_global_localization'


def read_json(path):
    if path.stat().st_size > 1024 * 1024:
        raise ValueError(f'VGL metadata too large: {path}')
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def model_onnx_identity(folder):
    folder = Path(folder)
    # Lab exports retain the ONNX and manifest alongside runtime_models.
    run = folder.parent if folder.name == 'runtime_models' else folder
    manifest = run / 'manifest.json'
    if manifest.is_file():
        data = read_json(manifest)
        onnx = run / 'aliked.onnx'
        expected = data.get('onnx_sha256')
        if expected and onnx.is_file() and digest(onnx) == expected:
            return expected
    onnx = folder / 'aliked_lightglue/aliked.onnx'
    return digest(onnx) if onnx.is_file() else None


def resolve_profile(map_dir, model_arg, config_arg, default_config, roots):
    """Explicit arguments win. Profiled maps never fall back to the large model."""
    auto_model = str(model_arg) in ('', 'auto')
    auto_config = str(config_arg) in ('', 'auto')
    root = Path(map_dir)
    if root.name == 'cuvgl_map':
        root = root.parent
    profile_path = root / 'vgl_profile.json'
    if not profile_path.is_file():
        return (DEFAULT_MODEL if auto_model else str(model_arg),
                str(default_config) if auto_config else str(config_arg),
                'Legacy map without VGL profile; using explicit/default settings')
    profile = read_json(profile_path)
    if profile.get('status') != 'complete':
        raise ValueError(f'VGL map profile is not complete: {profile_path}')
    config = Path(config_arg) if not auto_config else root / profile.get('runtime_config', 'vgl_runtime_config')
    if auto_config and not config.resolve().is_relative_to(root.resolve()):
        raise ValueError('VGL runtime config must be inside its map folder')
    if not (config / 'keypoint_creation_config.pb.txt').is_file():
        raise ValueError(f'Map VGL runtime config missing: {config}')
    if auto_config:
        text = (config / 'keypoint_creation_config.pb.txt').read_text()
        detector = text.split('aliked_detector', 1)[-1].split('super_point_detector', 1)[0]
        shape = [int(v) for v in re.findall(r'\bopt\s*:\s*(\d+)', detector)]
        if shape != [1, 3, profile['height'], profile['width']]:
            raise ValueError('Map runtime config input size differs from VGL profile')
    if not auto_model:
        return str(model_arg), str(config), 'Explicit VGL model override'
    recorded = Path(profile.get('model_dir', ''))
    wanted = profile.get('onnx_sha256')
    candidates = [recorded]
    if wanted:
        for search_root in map(Path, roots):
            if search_root.is_dir():
                for child in sorted(search_root.iterdir()):
                    candidates.extend([child, child / 'runtime_models'])
    matches = []
    for candidate in candidates:
        if not candidate.is_dir() or candidate.resolve() in matches:
            continue
        engines = candidate / 'aliked_lightglue'
        if not list(engines.glob('aliked_*.engine')) or not list(engines.glob('lightglue_aliked_*.engine')):
            continue
        if wanted:
            try:
                if model_onnx_identity(candidate) != wanted:
                    continue
            except (ValueError, OSError):
                continue
        else:
            # Older maps recorded only the build engine. Cross-host use is limited
            # to the exact recorded path and requires lab shape metadata.
            inspection = candidate.parent / 'engine-inspection.json'
            if not inspection.is_file():
                continue
            data = read_json(inspection)
            shape = next((t.get('shape') for t in data.get('tensors', []) if t.get('name') == 'image'), None)
            if shape != [1, 3, profile['height'], profile['width']]:
                continue
            engine_file = engines / Path(data.get('file', '')).name
            if not engine_file.is_file() or digest(engine_file) != data.get('sha256'):
                continue
        matches.append(candidate.resolve())
        if candidate == recorded:
            break
    if len(matches) != 1:
        raise ValueError(f'Cannot select VGL model for {root}: {len(matches)} matching folders. '
                         'Install the matching ONNX and locally built engines, or set vgl_model_dir explicitly.')
    note = 'VGL model selected by ONNX SHA-256' if wanted else 'Legacy VGL profile: recorded path and input size matched; ONNX identity not recorded'
    return str(matches[0]), str(config), note
