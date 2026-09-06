#!/usr/bin/env python3
"""Build poses with Isaac ROS, then extract VGL features with an explicit profile."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess

from configure_vgl_extractor import configure


def positive(value):
    number = int(value)
    if not 1 <= number <= 8192:
        raise argparse.ArgumentTypeError('image size must be between 1 and 8192')
    return number


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_engine(model_dir, width, height):
    paths = list((model_dir / 'aliked_lightglue').glob('aliked_*.engine'))
    if len(paths) != 1:
        raise ValueError('Select a model folder with exactly one ALIKED engine built on this GPU')
    import tensorrt as trt
    logger = trt.Logger(trt.Logger.WARNING)
    trt.init_libnvinfer_plugins(logger, '')
    with trt.Runtime(logger) as runtime:
        engine = runtime.deserialize_cuda_engine(paths[0].read_bytes())
        if engine is None:
            raise ValueError('Cannot load ALIKED engine; build it on the mapping GPU')
        shape = list(engine.get_tensor_shape('image'))
        if shape != [1, 3, height, width]:
            raise ValueError(f'ALIKED input {shape} differs from requested [1, 3, {height}, {width}]')
        del engine
    if not list((model_dir / 'aliked_lightglue').glob('lightglue_aliked_*.engine')):
        raise ValueError('LightGlue ALIKED engine is missing')
    return {'aliked_engine': paths[0].name, 'aliked_sha256': sha256(paths[0]), 'input_shape': shape}


def require_options(help_text, options):
    available = set(re.findall(r'--[A-Za-z_][A-Za-z_0-9-]*', help_text))
    missing = set(options) - available
    if missing:
        raise ValueError('Installed create_cuvgl_map.py lacks required options: ' + ', '.join(sorted(missing)))


def create_vgl_command(map_dir, frames, config_dir, model_dir, binary_dir):
    return ['ros2', 'run', 'isaac_ros_visual_mapping', 'create_cuvgl_map.py',
            f'--map_folder={map_dir}', f'--raw_image_folder={frames}',
            f'--config_folder_path={config_dir}', f'--model_dir={model_dir}',
            f'--binary_folder_path={binary_dir}', '--feature_type=aliked',
            '--extract_feature', '--build_bow_index']


def build(args):
    steps = args.steps_to_run
    if len(set(steps)) != len(steps):
        raise ValueError('Duplicate mapping steps')
    if args.width * args.height < 2048:
        raise ValueError('ALIKED input must contain at least 2048 pixels')
    base = args.base_output_folder.resolve()
    model = args.model_dir.resolve()
    # Check before running expensive pose generation. Never silently use a default model.
    if 'cuvgl' in steps:
        if steps[-1] != 'cuvgl':
            raise ValueError('cuvgl must be the last mapping step')
        help_text = subprocess.check_output(
            ['ros2', 'run', 'isaac_ros_visual_mapping', 'create_cuvgl_map.py', '--help'], text=True)
        require_options(help_text, ['--map_folder', '--raw_image_folder', '--config_folder_path',
                                   '--model_dir', '--binary_folder_path', '--feature_type',
                                   '--extract_feature', '--build_bow_index'])
        engine_record = check_engine(model, args.width, args.height)
        prefix = Path(subprocess.check_output(
            ['ros2', 'pkg', 'prefix', 'isaac_ros_visual_mapping'], text=True).strip())
        share = Path(subprocess.check_output(
            ['ros2', 'pkg', 'prefix', '--share', 'isaac_ros_visual_mapping'], text=True).strip())
        launch_share = Path(subprocess.check_output(
            ['ros2', 'pkg', 'prefix', '--share', 'jetpilot_system_launch'], text=True).strip())
        config_source = share / 'configs/isaac'
        # Validate the textproto layout before any map processing.
        configure((config_source / 'keypoint_creation_config.pb.txt').read_text(), args.width, args.height)
    pose_steps = [s for s in steps if s != 'cuvgl']
    if not pose_steps:
        raise ValueError('For a new map, include edex and compute_poses before cuvgl')
    base.mkdir(parents=True, exist_ok=True)
    before = set(base.iterdir())
    command = ['ros2', 'run', 'isaac_mapping_ros', 'create_map_offline.py',
               f'--sensor_data_bag={args.sensor_data_bag}', f'--base_output_folder={base}',
               f'--camera_topic_config={args.camera_topic_config}', f'--fs_model_res={args.fs_model_res}',
               '--steps_to_run', *pose_steps]
    print('[stage] create poses: ' + ' '.join(command), flush=True)
    subprocess.run(command, check=True)
    if 'cuvgl' not in steps:
        return
    candidates = [p for p in set(base.iterdir()) - before
                  if (p / 'map_frames/rectified/frames_meta.json').is_file()]
    if len(candidates) != 1:
        raise ValueError(f'Expected one new map with rectified frames; found {len(candidates)}')
    generated = candidates[0]
    vgl_map = generated / 'cuvgl_map'
    if vgl_map.exists():
        raise ValueError(f'Refusing to reuse existing VGL features/index: {vgl_map}')
    mapping_config = generated / 'vgl_mapping_config'
    runtime_config = generated / 'vgl_runtime_config'
    for source, target in [(config_source, mapping_config),
                           (launch_share / 'config/localization/vgl_config', runtime_config)]:
        shutil.copytree(source, target, symlinks=False)
        config = target / 'keypoint_creation_config.pb.txt'
        config.write_text(configure(config.read_text(), args.width, args.height))
    record = dict(engine_record, model_dir=str(model), width=args.width, height=args.height,
                  mapping_config='vgl_mapping_config', runtime_config='vgl_runtime_config',
                  status='building', vocabulary='new; no prebuilt vocabulary supplied')
    record_path = generated / 'vgl_profile.json'
    record_path.write_text(json.dumps(record, indent=2) + '\n')
    vgl_map.mkdir()
    command = create_vgl_command(vgl_map, generated / 'map_frames/rectified', mapping_config,
                                 model, prefix / 'bin/visual_mapping')
    print('[stage] create VGL features and index: ' + ' '.join(command), flush=True)
    subprocess.run(command, check=True)
    for artifact in ['keyframes/frames_meta.json', 'bow_index.pb', 'vocabulary']:
        if not (vgl_map / artifact).exists():
            raise ValueError(f'VGL output missing: {artifact}')
    record['status'] = 'complete'
    record_path.write_text(json.dumps(record, indent=2) + '\n')
    print(f'VGL map: {vgl_map}\nUse vgl_config_dir:={runtime_config}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sensor_data_bag', type=Path, required=True)
    parser.add_argument('--base_output_folder', type=Path, required=True)
    parser.add_argument('--camera_topic_config', type=Path, required=True)
    parser.add_argument('--fs_model_res', choices=['low_res', 'high_res'], default='low_res')
    parser.add_argument('--steps_to_run', nargs='+', default=['edex', 'compute_poses', 'cuvgl'],
                        choices=['edex', 'compute_poses', 'depth', 'occupancy', 'transform_map', 'cuvgl'])
    parser.add_argument('--model-dir', dest='model_dir', type=Path, required=True)
    parser.add_argument('--width', type=positive, default=1920)
    parser.add_argument('--height', type=positive, default=1200)
    args = parser.parse_args()
    try:
        build(args)
    except (ValueError, OSError, ImportError, subprocess.CalledProcessError) as error:
        parser.exit(1, f'ERROR: {error}\n')


if __name__ == '__main__':
    main()
