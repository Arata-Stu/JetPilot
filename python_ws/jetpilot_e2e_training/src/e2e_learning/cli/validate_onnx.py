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
    input_name = session.get_inputs()[0].name
    input_shape = [1, 3, int(cfg.data.input_height), int(cfg.data.input_width)]
    x = np.random.randn(*input_shape).astype(np.float32)
    outputs = session.run(None, {input_name: x})
    print(f"ONNX OK: {onnx_path}")
    print(f"input : {input_name} {input_shape}")
    print(f"output: {[list(out.shape) for out in outputs]}")


if __name__ == "__main__":
    main()
