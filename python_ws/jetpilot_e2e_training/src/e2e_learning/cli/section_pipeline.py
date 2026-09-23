"""Section multihead jobs. Heavy dependencies are imported only by the selected job."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

from e2e_learning.data.sections import SectionMap, map_digest, split_sequences, validate_heads

MEAN = [0.485, 0.456, 0.406]
STD = [0.229, 0.224, 0.225]


def write_json(path, value):
    path = Path(path)
    tmp = path.with_suffix(path.suffix+".tmp")
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))
    tmp.replace(path)


def read_rows(directory):
    with (Path(directory)/"samples.csv").open(newline="") as fp:
        return list(csv.DictReader(fp))


def extract_one(bag, directory, request):
    from e2e_learning.data.rosbag_extractor import ExtractConfig, extract_dataset
    return extract_dataset(ExtractConfig(
        bag_path=Path(bag), output_dir=Path(directory),
        image_topic=request.get("image_topic", "/realsense/color/image_raw"),
        control_topic=request.get("control_topic", "/teleop/control_cmd"),
        input_width=212, input_height=120,
        timestamp_source=request.get("timestamp_source", "header"),
        max_control_dt_sec=request.get("max_control_dt_sec", 0.1),
    ))


def preprocess(request):
    from e2e_learning.data.bag_sections import recorded_labels
    geometry = SectionMap(request["map_document"], request.get("max_lane_distance_m", 1.0))
    selected = set(request["sections"])
    if not selected or not selected <= set(geometry.ids):
        raise ValueError("抽出対象のsectionを選択してください")
    output = Path(request["output"])
    output.mkdir(parents=True, exist_ok=False)
    (output/"images").mkdir()
    rows, counts, dropped, section_counts = [], {}, {}, {}
    for bag in request["rosbags"]:
        print(f"Extracting {bag}", flush=True)
        with tempfile.TemporaryDirectory(prefix="section-extract-", dir=output) as work:
            extract_one(bag, work, request)
            source = read_rows(work)
            if request["mode"] == "offline":
                trace = Path(work)/"sections.json"
                from rosbags.highlevel import AnyReader
                with AnyReader([Path(bag)]) as reader:
                    sensor_types = {"sensor_msgs/msg/Image", "sensor_msgs/msg/CameraInfo", "sensor_msgs/msg/Imu"}
                    replay_topics = sorted({c.topic for c in reader.connections if c.msgtype in sensor_types or c.topic == "/planning/current_lane"})
                    replay_duration = (reader.end_time-reader.start_time)/1e9
                replay_request = {**request, "rosbag": bag, "trace_path": str(trace), "replay_topics": replay_topics, "replay_duration_sec": replay_duration}
                request_file = Path(work)/"replay.json"
                write_json(request_file, replay_request)
                subprocess.run(["/usr/bin/python3", "-m", "e2e_learning.cli.offline_sections", "--request", str(request_file)], check=True)
                labels = {int(k): v for k,v in json.loads(trace.read_text()).items()}
            else:
                labels = recorded_labels(bag, source, request)
            kept = 0
            sequence = hashlib.sha256(str(Path(bag).resolve()).encode()).hexdigest()[:16]
            for row in source:
                section = labels.get(int(row["stamp"]), "unknown")
                if section not in selected:
                    dropped[section] = dropped.get(section, 0)+1
                    continue
                if not all(math.isfinite(float(row[key])) for key in ("steering", "throttle")):
                    dropped["non_finite_control"] = dropped.get("non_finite_control", 0)+1
                    continue
                # Optional filtering is disabled for transition recordings.
                expected = request.get("filter_throttle")
                if expected is not None and abs(float(row["throttle"])-expected) > request.get("throttle_tolerance", 0.015):
                    dropped["throttle_mismatch"] = dropped.get("throttle_mismatch", 0)+1
                    continue
                relative = Path("images")/f"{len(rows):09d}.jpg"
                shutil.copyfile(Path(work)/row["image_path"], output/relative)
                rows.append({**row, "image_path": relative.as_posix(), "sequence_id": sequence,
                             "section_id": section, "source_bag": str(bag)})
                kept += 1
                section_counts[section] = section_counts.get(section, 0)+1
            counts[str(bag)] = kept
    if not rows:
        raise RuntimeError("対象sectionの有効サンプルが0件です。地図・topic・時刻・推定品質を確認してください")
    with (output/"samples.csv").open("w", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    metadata = {"schema_version": 1, "kind": "section_dataset", "task": "control",
                "input_width": 212, "input_height": 120, "sample_count": len(rows),
                "map": request["map"], "map_sha256": map_digest(request["map_document"]),
                "map_document": request["map_document"], "sections": sorted(selected),
                "source_bags": request["rosbags"], "bag_counts": counts, "section_counts": section_counts, "dropped": dropped,
                "assignment": request["mode"], "timestamp_source": request.get("timestamp_source", "header"),
                "filter_throttle": request.get("filter_throttle"),
                "recommended_throttle": request["throttle"], "request": request}
    write_json(output/"section_dataset.json", metadata)
    # JSON is also valid YAML and is readable by existing dataset scanners.
    write_json(output/"metadata.yaml", metadata)
    print(f"Dataset complete: {len(rows)} samples, {counts}", flush=True)


def train(request):
    import copy
    import torch
    from torch.utils.data import DataLoader, Subset
    from e2e_learning.data.dataset import E2EDataset
    from e2e_learning.models.section_multihead import SectionMultiheadControl
    from e2e_learning.models.dinov3_vit import load_dinov3_backbone_weights
    heads = request["heads"]
    datasets = [json.loads((Path(h["dataset"])/"section_dataset.json").read_text()) for h in heads]
    if len({d["map_sha256"] for d in datasets}) != 1:
        raise ValueError("全headのdatasetは同じ地図・section定義で作成してください")
    geometry = SectionMap(datasets[0]["map_document"])
    validate_heads(heads, geometry.ids)
    for head, dataset in zip(heads, datasets):
        if set(head["sections"]) != set(dataset["sections"]):
            raise ValueError(f"{head['name']}: datasetのsectionとhead設定が一致しません")
        if head.get("generic") and set(head["sections"]) != set(geometry.ids):
            raise ValueError("汎用headには全sectionを選択したdatasetが必要です")
    output = Path(request["output"])
    output.mkdir(parents=True, exist_ok=False)
    (output/"checkpoints").mkdir()
    torch.manual_seed(42)
    device = torch.device(request.get("device", "cuda"))
    model = SectionMultiheadControl([h["name"] for h in heads])
    load_dinov3_backbone_weights(request["backbone_weights"], model.backbone, require_all_parameters=True)
    model.to(device)
    loaders, validations, splits = {}, {}, {}
    # Heads have independent parameters and the shared backbone is frozen.
    # Split each head's own bags; different throttle datasets need not share bags.
    for head in heads:
        name = head["name"]
        ds = E2EDataset(head["dataset"], 212, 120, tuple(MEAN), tuple(STD))
        training, validation, split_kind = split_sequences(ds.rows)
        if not training or not validation:
            raise ValueError(f"{name}: 学習・検証に必要なサンプルが不足しています")
        options = dict(batch_size=request.get("batch_size", 32), num_workers=request.get("workers", 4))
        loaders[name] = DataLoader(Subset(ds, training), shuffle=True, **options)
        validations[name] = DataLoader(Subset(ds, validation), shuffle=False, **options)
        splits[name] = {"kind": split_kind, "train": len(training), "validation": len(validation),
                        "train_bags": sorted({ds.rows[i]["sequence_id"] for i in training}),
                        "validation_bags": sorted({ds.rows[i]["sequence_id"] for i in validation})}
    optimizers = {name: torch.optim.AdamW(model.heads[name].parameters(), lr=request.get("learning_rate", 0.001)) for name in model.head_names}
    history, best, best_weights = [], {name: float("inf") for name in model.head_names}, {}
    for epoch in range(request.get("epochs", 30)):
        model.train()
        iterators = {name: iter(loader) for name, loader in loaders.items()}
        totals = {name: [0., 0] for name in model.head_names}
        while iterators:
            for name in list(iterators):
                batch = next(iterators[name], None)
                if batch is None:
                    del iterators[name]
                    continue
                images, _, targets = batch
                with torch.no_grad():
                    features = model.features(images[:, 0].to(device))
                prediction = model.heads[name](features)
                target = targets[:, :1].to(device)
                loss = torch.nn.functional.mse_loss(prediction, target)
                if not torch.isfinite(loss):
                    raise RuntimeError(f"{name}: non-finite training loss")
                optimizers[name].zero_grad(set_to_none=True)
                loss.backward()
                optimizers[name].step()
                totals[name][0] += loss.item()*len(images)
                totals[name][1] += len(images)
        model.eval()
        record = {"epoch": epoch+1, "heads": {}}
        with torch.no_grad():
            for name, loader in validations.items():
                error, count = 0., 0
                for images, _, targets in loader:
                    pred = model.heads[name](model.features(images[:, 0].to(device)))
                    error += (pred-targets[:, :1].to(device)).abs().sum().item()
                    count += len(images)
                mae = error/count
                if not math.isfinite(mae):
                    raise RuntimeError("non-finite validation error")
                record["heads"][name] = {"train_mse": totals[name][0]/totals[name][1], "validation_mae": mae}
                if mae < best[name]:
                    best[name] = mae
                    best_weights[name] = copy.deepcopy({k:v.cpu() for k,v in model.heads[name].state_dict().items()})
        history.append(record)
        write_json(output/"metrics.json", {"history": history, "best_mae": best, "splits": splits})
        print(json.dumps(record), flush=True)
    for name in model.head_names:
        model.heads[name].load_state_dict(best_weights[name])
    metadata = {"schema_version": 1, "model_kind": "section_multihead", "model_name": output.name,
                "task": "control", "modality": "rgb", "steering_only": True, "backbone_frozen": True,
                "heads": heads, "generic_head": next(h["name"] for h in heads if h.get("generic")),
                "map": datasets[0]["map"], "map_sha256": datasets[0]["map_sha256"],
                "map_document": datasets[0]["map_document"],
                "section_policy": {s: {"head": h["name"], "throttle": h["throttle"]} for h in heads if not h.get("generic") for s in h["sections"]},
                "input": {"name": "image", "shape": [1,3,120,212], "mean": MEAN, "std": STD},
                "output": {"name": "control", "shape": [1,len(heads)], "head_order": model.head_names, "learned_fields": ["steering"]},
                "best_mae": best, "splits": splits, "training_request": request}
    torch.save({"model_state": model.cpu().state_dict(), "metadata": metadata}, output/"checkpoints"/"best.pt")
    write_json(output/"metadata.json", metadata)
    export({"run": str(output)})


def export(request):
    import torch
    from e2e_learning.models.section_multihead import SectionMultiheadControl
    output = Path(request["run"])
    checkpoint = torch.load(output/"checkpoints"/"best.pt", map_location="cpu", weights_only=True)
    metadata = checkpoint["metadata"]
    model = SectionMultiheadControl(metadata["output"]["head_order"])
    model.load_state_dict(checkpoint["model_state"], strict=True)
    model.eval()
    tmp = output/"model.building.onnx"
    torch.onnx.export(model, torch.zeros(1,3,120,212), str(tmp), input_names=["image"],
                      output_names=["control"], opset_version=17, dynamo=False)
    import onnx
    onnx.checker.check_model(onnx.load(str(tmp)))
    import numpy as np
    import onnxruntime as ort
    sample = np.random.default_rng(42).normal(size=(1,3,120,212)).astype(np.float32)
    with torch.no_grad():
        reference = model(torch.from_numpy(sample)).numpy()
    actual = ort.InferenceSession(str(tmp), providers=["CPUExecutionProvider"]).run(None, {"image":sample})[0]
    np.testing.assert_allclose(actual, reference, rtol=1e-3, atol=1e-4)
    tmp.replace(output/"model.onnx")
    metadata["onnx_verified"] = True
    write_json(output/"metadata.json", metadata)
    print(f"ONNX verified: {output/'model.onnx'}", flush=True)


def analyze(request):
    import cv2
    import numpy as np
    import onnxruntime as ort
    from e2e_learning.data.transforms import ImageTransform
    run = Path(request["run"])
    metadata = json.loads((run/"metadata.json").read_text())
    index = metadata["output"]["head_order"].index(request["head"])
    transform = ImageTransform(212,120,tuple(MEAN),tuple(STD))
    session = ort.InferenceSession(str(run/"model.onnx"), providers=["CPUExecutionProvider"])
    output = Path(request["output"])
    output.mkdir(parents=True, exist_ok=False)
    preview, count, absolute, squared, maximum = [], 0, 0., 0., 0.
    preview_stride = max(1, request.get("preview_stride", 10))
    with (output/"predictions.csv").open("w", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=["sample_index", "bag", "stamp", "head", "target", "prediction", "error", "recorded_throttle"])
        writer.writeheader()
        for bag in request["rosbags"]:
            with tempfile.TemporaryDirectory(prefix="head-eval-", dir=output) as work:
                extract_one(bag, work, request)
                for row in read_rows(work):
                    image = cv2.imread(str(Path(work)/row["image_path"]))
                    if image is None:
                        raise RuntimeError("解析画像を読み込めません")
                    prediction = float(session.run(None, {"image": transform(image)[None]})[0][0,index])
                    target = float(row["steering"])
                    error = prediction-target
                    if not math.isfinite(error):
                        raise RuntimeError("non-finite inference result")
                    record = {"sample_index":count, "bag":str(bag), "stamp":row["stamp"], "head":request["head"], "target":target,
                              "prediction":prediction, "error":error, "recorded_throttle":float(row["throttle"])}
                    writer.writerow(record)
                    if count % preview_stride == 0:
                        preview.append(record)
                        if len(preview) > 4000:
                            preview_stride *= 2
                            preview = [p for p in preview if p["sample_index"] % preview_stride == 0]
                    count += 1
                    absolute += abs(error)
                    squared += error**2
                    maximum = max(maximum, abs(error))
    if not count:
        raise RuntimeError("解析対象サンプルがありません")
    write_json(output/"report.json", {"head":request["head"], "run":str(run), "count":count,
                                      "mae":absolute/count, "rmse":math.sqrt(squared/count),
                                      "max_abs":maximum, "preview":preview,
                                      "scope":"selected head on all frames; no automatic routing",
                                      "request":request})
    print(f"Analysis complete: {count} samples, MAE={absolute/count:.6f}", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=["preprocess", "train", "export", "analyze"])
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--request-json")
    group.add_argument("--request-file")
    args = parser.parse_args()
    raw = Path(args.request_file).read_text() if args.request_file else args.request_json
    globals()[args.action](json.loads(raw))


if __name__ == "__main__":
    main()
