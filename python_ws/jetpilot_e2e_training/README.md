# jetpilot_e2e_training

Image-based E2E control training tools for JetPilot.

The first target is a two-channel control output:

- `steering`: `[-1.0, 1.0]`
- `throttle`: `[0.0, 1.0]`

The package keeps the training path and the Jetson TensorRT path connected through
`metadata.json`. The default model is a small PilotNet-style CNN trained from
scratch. Pretrained encoder experiments can be enabled later with the same trainer.
MobileNet presets use torchvision pretrained weights by default; set
`model.pretrained=false` or provide `model.weights_path=/path/to/weights.pth` when
working offline.

## Typical Flow

```bash
source /opt/env/bin/activate
cd python_ws/jetpilot_e2e_training

scripts/train_interactive.sh
```

Manual flow:

```bash
python -m e2e_learning.cli.preprocess_bag \
  data.bag_path=/bags/run_001 \
  data.image_topic=/realsense/color/image_raw \
  data.control_topic=/teleop/control_cmd \
  data.output_dir=datasets/run_001

python -m e2e_learning.cli.train \
  experiment=pilotnet_scratch \
  data.dataset_dir=datasets/run_001 \
  run.name=pilotnet_run_001

python -m e2e_learning.cli.export_onnx \
  checkpoint=outputs/e2e/pilotnet_run_001/checkpoints/best.pt
```

## Comparison Experiments

```bash
scripts/run_experiments.sh datasets/run_001 exp_run_001
scripts/compare_runs.sh outputs/e2e_experiments/exp_run_001
```

The interactive scripts keep generated data inside this package by default:

- datasets: `datasets/<dataset-name>`
- training outputs: `outputs/e2e/<run-name>`

Override these locations with `JETPILOT_E2E_DATASET_ROOT` and
`JETPILOT_E2E_OUTPUT_ROOT` when needed.

Each run writes UI-friendly files:

- `run.yaml`
- `progress.json`
- `metrics.json`
- `checkpoints/best.pt`
- `model.onnx`
- `metadata.json`

## Jetson Deploy and TensorRT

```bash
scripts/deploy_model.sh outputs/e2e/pilotnet_run_001/model.onnx --preset camera_control
```

On Jetson:

```bash
ros2 run jetpilot_e2e_inference build_tensorrt_engine.sh \
  /opt/jetpilot/models/e2e/camera_control/model.onnx \
  /opt/jetpilot/models/e2e/camera_control/model.plan
```
