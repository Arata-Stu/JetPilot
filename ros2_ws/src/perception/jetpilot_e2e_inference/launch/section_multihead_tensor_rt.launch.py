"""Metadata-driven shared DINOv3 inference and guarded section routing."""
import hashlib
import json
from pathlib import Path
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    root = Path(get("model_root"))
    metadata = json.loads((root/"metadata.json").read_text())
    if metadata.get("model_kind") != "section_multihead" or not metadata.get("onnx_verified"):
        raise ValueError("A verified section_multihead model is required")
    document = metadata["map_document"]
    digest = hashlib.sha256(json.dumps(document, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()
    if digest != metadata["map_sha256"]:
        raise ValueError("Embedded section map checksum mismatch")
    localization_map = get("localization_map")
    if not localization_map:
        raise ValueError("localization_map is required to verify the section coordinate system")
    if localization_map:
        import yaml
        directory = Path(localization_map)
        map_path = directory / f"{directory.name}_hd_map.yaml"
        if not map_path.is_file():
            map_path = directory / "hd_map.yaml"
        if not map_path.is_file():
            raise ValueError("Localization map must contain its matching HD map YAML")
        live_document = yaml.safe_load(map_path.read_text())
        live_digest = hashlib.sha256(json.dumps(live_document, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()
        if live_digest != digest:
            raise ValueError("Localization map sections differ from the trained map")
    heads, order, policy = metadata["heads"], metadata["output"]["head_order"], metadata["section_policy"]
    if [h["name"] for h in heads] != order or len(set(order)) != len(order):
        raise ValueError("Invalid head order")
    generic = next(h for h in heads if h["name"] == metadata["generic_head"])
    sim = get("use_sim_time").lower() == "true"
    extra = [{"use_intra_process_comms":True}]
    encoder = ComposableNode(package="isaac_ros_dnn_image_encoder",
        plugin="nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode", name="section_image_encoder",
        parameters=[{"input_qos":"SENSOR_DATA", "input_image_width":int(get("input_image_width")),
                     "input_image_height":int(get("input_image_height")), "network_image_width":212,
                     "network_image_height":120, "input_encoding":"rgb8", "enable_padding":False,
                     "image_mean":metadata["input"]["mean"], "image_stddev":metadata["input"]["std"],
                     "tensor_name":"input_tensor", "use_sim_time":sim}],
        remappings=[("image",get("image_topic")),("camera_info",get("camera_info_topic")),("tensors","/e2e/section/input")], extra_arguments=extra)
    trt = ComposableNode(package="isaac_ros_tensor_rt", plugin="nvidia::isaac_ros::dnn_inference::TensorRTNode", name="section_tensor_rt",
        parameters=[{"model_file_path":str(root/"model.onnx"), "engine_file_path":str(root/"model.plan"),
                     "force_engine_update":False,"enable_fp16":True,"input_tensor_names":["input_tensor"],
                     "input_binding_names":["image"],"output_tensor_names":["output_tensor"],"output_binding_names":["control"],
                     "input_tensor_formats":["nitros_tensor_list_nchw_rgb_f32"],"output_tensor_formats":["nitros_tensor_list_nchw_rgb_f32"],"use_sim_time":sim}],
        remappings=[("tensor_pub","/e2e/section/input"),("tensor_sub","/e2e/section/output")], extra_arguments=extra)
    decoder = ComposableNode(package="jetpilot_e2e_inference", plugin="jetpilot_e2e_inference::E2EControlDecoderNode", name="e2e_control_decoder",
        parameters=[{"section_head_names":order, "section_ids":list(policy),
                     "section_head_indices":[order.index(p["head"]) for p in policy.values()],
                     "section_throttles":[float(p["throttle"]) for p in policy.values()],
                     "generic_head_index":order.index(generic["name"]), "generic_throttle":float(generic["throttle"]),
                     "section_topic":"/e2e/validated_section", "use_sim_time":sim}],
        remappings=[("tensor_sub","/e2e/section/output"),("control_cmd",get("control_cmd_topic"))], extra_arguments=extra)
    # The localizer consumes the exact section definition embedded during training.
    temporary = tempfile.TemporaryDirectory(prefix="jetpilot-section-map-")
    map_path = Path(temporary.name)/"hd_map.yaml"
    map_path.write_text(json.dumps(document))
    localizer = Node(package="jetpilot_hdmap_publisher", executable="hd_map_section_localizer_node.py", name="hd_map_section_localizer",
                     parameters=[{"hd_map_yaml_path":str(map_path),"use_sim_time":sim}])
    guard = Node(package="jetpilot_e2e_inference",executable="section_control_guard_node.py",
                 parameters=[{"map_frame":str(document.get("frame_id","map")),"use_sim_time":sim}])
    def cleanup(context):
        temporary.cleanup()
        return []
    actions = [localizer, guard, RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)]))]
    if get("run_standalone").lower() == "true":
        actions.append(ComposableNodeContainer(name=get("container_name"), namespace="", package="rclcpp_components",
                       executable="component_container_mt", composable_node_descriptions=[encoder,trt,decoder], output="screen"))
    else:
        actions.append(LoadComposableNodes(target_container=get("container_name"), composable_node_descriptions=[encoder,trt,decoder]))
    return actions


def generate_launch_description():
    defaults = {"model_root":"", "localization_map":"", "image_topic":"/realsense/color/image_raw", "camera_info_topic":"/realsense/color/camera_info",
                "input_image_width":"424", "input_image_height":"240", "control_cmd_topic":"/auto/control_cmd",
                "container_name":"section_multihead_container", "run_standalone":"true", "use_sim_time":"false"}
    return LaunchDescription([*(DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()),OpaqueFunction(function=setup)])
