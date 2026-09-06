from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults={'detections_topic':'/perception/detections','camera_info_topic':'/realsense/color/camera_info',
              'map_frame':'map','image_geometry':'raw','source_width':'424','source_height':'240',
              'max_range_m':'8.0','use_sim_time':'false'}
    parameters={key:LaunchConfiguration(key) for key in defaults}
    for key,kind in [('source_width',int),('source_height',int),('max_range_m',float),('use_sim_time',bool)]:
        parameters[key]=ParameterValue(LaunchConfiguration(key),value_type=kind)
    return LaunchDescription([DeclareLaunchArgument(key,default_value=value) for key,value in defaults.items()]+[
        Node(package='jetpilot_object_detection',executable='opponent_projection_node.py',
             name='opponent_projection',parameters=[parameters],output='screen')])
