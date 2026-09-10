# Local foundation-model weights

This directory only defines local destinations. Model files are intentionally
ignored by Git and must be obtained by the user under their respective licenses.

## RGB: official DINOv3

Place the authorized DINOv3 ViT-S/16 backbone at:

```text
weights/dinov3/dinov3_vits16_pretrain_lvd1689m-08c60483.pth
```

`experiment=dinov3_vits16_finetune` uses this path by default. Set
`DINOV3_VITS16_WEIGHTS` to use a different local path.

## EVS: GEP event encoder

Place a DINOv3-based Generative Event Pretraining event-encoder checkpoint under:

```text
weights/gep/
```

The Event Encoder Small/Base weights accompanying the published GEP method are
DINOv2-initialized and are not compatible with this DINOv3 model. Use this
directory only for a separately trained or newly published DINOv3-based GEP
checkpoint. A compatible DINOv3 ViT-S/16 event encoder contains
`rope_embed.periods` and `storage_tokens`, uses patch size 16, embed dimension
384, and depth 12. Select it explicitly during training:

```bash
python -m e2e_learning.cli.train \
  experiment=dinov3_vits16_finetune \
  model.weights_path=weights/gep/<downloaded-checkpoint>.pt \
  data.dataset_dir=datasets/<evs-dataset> \
  run.name=dinov3_vits16_evs
```

The loader accepts a raw DINOv3 state dictionary or a GEP checkpoint containing
an `event_encoder` state dictionary. Incompatible DINOv2 checkpoints are rejected.

## EVS: EventState DINOv3 encoder

Place the complete EventState checkpoint at:

```text
weights/eventstate/eventstate_dinov3_vits16.pth
```

`experiment=dinov3_vits16_eventstate` extracts
`model.event_encoder.backbone.*` from the complete checkpoint. The preset
assumes EventState's default three-channel `gep_rgb` representation and DSEC
event mean/std. If the checkpoint was trained with another representation,
window duration, or statistics, copy those values from its embedded `config`
instead of using the defaults. A 20-channel voxel checkpoint is not an
input-compatible replacement for this three-channel preset.
