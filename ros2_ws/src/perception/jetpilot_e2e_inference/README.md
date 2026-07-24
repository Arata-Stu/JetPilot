# jetpilot_e2e_inference

ROS 2 inference nodes for JetPilot image-based E2E control.

Pipeline:

```text
sensor_msgs/Image
  -> e2e_image_encoder
  -> isaac_ros_tensor_rt
  -> e2e_control_decoder
  -> jetpilot_msgs/ControlCommand
```

The exported ONNX model must use these binding names by default:

- input: `image`
- output: `control`

TensorRT topic tensor names:

- input tensor: `input_tensor`
- output tensor: `output_tensor`

Build the TensorRT engine on Jetson:

```bash
ros2 run jetpilot_e2e_inference build_tensorrt_engine.sh \
  /opt/jetpilot/models/e2e/latest/model.onnx \
  /opt/jetpilot/models/e2e/latest/model.plan
```

Launch inference:

```bash
ros2 launch jetpilot_e2e_inference e2e_tensor_rt.launch.py \
  image_topic:=/realsense/color/image_raw \
  control_cmd_topic:=/auto/control_cmd \
  model_root:=/opt/jetpilot/models/e2e/latest
```
