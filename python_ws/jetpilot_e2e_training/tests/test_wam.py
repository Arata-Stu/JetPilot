import unittest

import torch
from torch import nn

from e2e_learning.models.wam import DinoV3TinyWAM, WAMTensorRTWrapper


class TinyWAMTests(unittest.TestCase):
    def _model(self) -> DinoV3TinyWAM:
        return DinoV3TinyWAM(
            input_height=32,
            input_width=32,
            patch_size=16,
            embed_dim=32,
            depth=1,
            num_heads=4,
            storage_tokens=1,
            latent_dim=16,
            hidden_dim=12,
            future_horizon=3,
        ).eval()

    def test_training_and_stateful_inference_shapes(self) -> None:
        model = self._model()
        images = torch.randn(2, 2, 3, 32, 32)
        actions = torch.zeros(2, 2, 2)
        future_actions = torch.zeros(2, 3, 2)

        result = model.forward_train(images, actions, actions[:, -1], future_actions)

        self.assertEqual(tuple(result["control"].shape), (2, 2))
        self.assertEqual(tuple(result["future_controls"].shape), (2, 3, 2))
        self.assertEqual(tuple(result["future_latents"].shape), (2, 3, 32))
        self.assertEqual(tuple(result["next_hidden"].shape), (2, 12))

        wrapper = WAMTensorRTWrapper(model)
        outputs = wrapper(torch.randn(1, 3, 32, 32), torch.zeros(1, 12), torch.zeros(1, 2))
        self.assertEqual([tuple(value.shape) for value in outputs], [
            (1, 2), (1, 12), (1, 3, 2), (1, 3, 32)
        ])

    def test_recurrent_cells_do_not_use_onnx_gru_operator(self) -> None:
        model = self._model()
        self.assertFalse(any(isinstance(module, (nn.GRU, nn.GRUCell)) for module in model.modules()))


if __name__ == "__main__":
    unittest.main()
