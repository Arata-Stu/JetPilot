# SPDX-License-Identifier: Apache-2.0
"""CAD mounting chain; the RealSense driver owns all D455 internal frames."""
import json
import math
from pathlib import Path


def build_transforms(config, base_frame='base_link', localization_frame='base_link',
                     camera_frame='realsense_camera_link', evs_frame='evs_link',
                     evs_optical_frame='event_camera', publish_evs=False):
    """Return validated parent/child/xyz/rpy records without importing ROS."""
    if config.get('schema_version') != 1:
        raise ValueError('TT-02 TF config schema_version must be 1')
    records = []

    def add(parent, child, transform):
        xyz, rpy = transform['xyz'], transform['rpy']
        if not all(isinstance(values, list) and len(values) == 3 and
                   all(isinstance(v, (int, float)) and not isinstance(v, bool) and
                       math.isfinite(v) for v in values) for values in (xyz, rpy)):
            raise ValueError(f'Invalid xyz/rpy for {parent} -> {child}')
        records.append((parent, child, xyz, rpy))

    identity = {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}
    if localization_frame != base_frame:
        if localization_frame != 'base_footprint':
            raise ValueError('TT-02 localization frame must be base_link (configured base) or base_footprint')
        height = config.get('base_height_m')
        if (not isinstance(height, (int, float)) or isinstance(height, bool)
                or not math.isfinite(height) or height <= 0):
            raise ValueError('Measure base_height_m before using base_footprint')
        add(localization_frame, base_frame, {'xyz': [0, 0, height], 'rpy': [0, 0, 0]})
    add(base_frame, 'tt02_plate_link', config['base_to_plate'])
    add('tt02_plate_link', 'camera_mount_link', config['plate_to_mount'])
    add('camera_mount_link', 'd455_mount_datum', config['mount_to_d455_datum'])
    add('camera_mount_link', 'd455_link', config['mount_to_d455'])
    if camera_frame != 'd455_link':
        add('d455_link', camera_frame, identity)
    if publish_evs:
        add('camera_mount_link', 'silky_mount_link', config['mount_to_silky'])
        add('silky_mount_link', 'silky_optical_frame', config['silky_to_optical'])
        # Body-axis mount alias and optical-axis message frame remain distinct.
        if evs_frame != 'silky_mount_link':
            add('silky_mount_link', evs_frame, identity)
        if evs_optical_frame != 'silky_optical_frame':
            add('silky_optical_frame', evs_optical_frame, identity)

    parents = {}
    for parent, child, _, _ in records:
        if any(not isinstance(frame, str) or not frame or frame.startswith('/') or
               any(c.isspace() for c in frame) for frame in (parent, child)):
            raise ValueError('TF frame names must be nonempty and have no leading slash/whitespace')
        if child in parents or child == localization_frame or child in ('map', 'odom'):
            raise ValueError(f'Duplicate or reserved TF child: {child}')
        parents[child] = parent
    for child in parents:
        visited = set()
        while child in parents:
            if child in visited:
                raise ValueError('Cycle in TT-02 TF tree')
            visited.add(child)
            child = parents[child]
    return records


def add_tt02(args):
    import isaac_ros_launch_utils as lu
    config = json.loads(Path(str(args.config_file)).expanduser().read_text(encoding='utf-8'))
    records = build_transforms(
        config, str(args.base_frame), str(args.localization_frame), str(args.camera_frame),
        str(args.evs_frame), str(args.evs_optical_frame), lu.is_true(args.publish_evs))
    actions = [lu.log_info(['TT-02 mounting TF config: ', str(args.config_file)])]
    for index, (parent, child, xyz, rpy) in enumerate(records):
        actions.append(lu.Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'tt02_mount_tf_{index}',
            arguments=[item for flag, value in zip(
                ('--x', '--y', '--z', '--roll', '--pitch', '--yaw'), xyz + rpy)
                for item in (flag, str(value))] + ['--frame-id', parent, '--child-frame-id', child],
            parameters=[{'use_sim_time': lu.is_true(args.use_sim_time)}], output='screen'))
    if not config['base_to_plate'].get('measured', False):
        actions.append(lu.log_info(['TT-02: base_link -> plate is provisional; measure rear axle to CAD origin.']))
    if lu.is_true(args.publish_evs) and not config['silky_to_optical'].get('measured', False):
        actions.append(lu.log_info(['TT-02: Silky optical centre uses provisional CAD/CS-mount values.']))
    return actions


def generate_launch_description():
    import isaac_ros_launch_utils as lu
    import isaac_ros_launch_utils.all_types as lut
    args = lu.ArgumentContainer()
    args.add_arg('config_file', lu.get_path('jetpilot_system_launch', 'config/vehicle/tt02_cad.json'), cli=True)
    args.add_arg('base_frame', 'base_link', cli=True)
    args.add_arg('localization_frame', 'base_link', cli=True)
    args.add_arg('camera_frame', 'realsense_camera_link', cli=True)
    args.add_arg('evs_frame', 'evs_link', cli=True)
    args.add_arg('evs_optical_frame', 'event_camera', cli=True)
    args.add_arg('publish_evs', False, cli=True)
    args.add_arg('use_sim_time', False, cli=True)
    args.add_opaque_function(add_tt02)
    return lut.LaunchDescription(args.get_launch_actions())
