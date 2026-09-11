from pathlib import Path

import hydra
import numpy as np
import onnx
import onnxruntime as ort
from omegaconf import DictConfig


@hydra.main(config_path="../conf", config_name="config", version_base=None)
def main(cfg: DictConfig) -> None:
    onnx_path = Path(str(cfg.onnx_path)).expanduser()
    if not onnx_path.exists():
        raise FileNotFoundError(f"ONNX not found: {onnx_path}")
    model = onnx.load(str(onnx_path))
    onnx.checker.check_model(model)
    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    session_inputs = session.get_inputs()
    input_shape = [1, 3, int(cfg.data.input_height), int(cfg.data.input_width)]
    feed = {session_inputs[0].name: np.random.randn(*input_shape).astype(np.float32)}
    for item in session_inputs[1:]:
        shape = tuple(int(value) if isinstance(value, int) and value > 0 else 1 for value in item.shape)
        feed[item.name] = np.zeros(shape, dtype=np.float32)
    outputs = session.run(None, feed)
    print(f"ONNX OK: {onnx_path}")
    print(f"inputs: {[(name, list(value.shape)) for name, value in feed.items()]}")
    print(f"output: {[list(out.shape) for out in outputs]}")


if __name__ == "__main__":
    main()
