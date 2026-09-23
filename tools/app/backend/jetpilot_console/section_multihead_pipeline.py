"""Console jobs for section datasets, frozen DINOv3 multihead training and deployment."""
from __future__ import annotations
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import shlex
import uuid

from .e2e_pipeline import PipelineTaskSpec, training_root, dataset_root, _name
from .map_formats import load_yaml
from .security import resolve_under_root, validate_ssh_target, validate_remote_absolute_path
from .shared_vit_pipeline import scan_backbone_weights


def roots(config):
    root = training_root(config)/"outputs"/"section_multihead"
    return dataset_root(config), root, root/"analyses"


def contract(config):
    path = training_root(config)/"src"/"e2e_learning"/"data"/"sections.py"
    spec = importlib.util.spec_from_file_location("section_contract", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def number(value, label, low, high):
    try:
        parsed = float(value)
    except (ValueError, TypeError):
        raise ValueError(f"{label}は数値で指定してください") from None
    if not math.isfinite(parsed) or not low <= parsed <= high:
        raise ValueError(f"{label}は{low}〜{high}の有限値が必要です")
    return parsed


def integer(value, label, low, high):
    parsed = number(value, label, low, high)
    if parsed != int(parsed):
        raise ValueError(f"{label}は整数で指定してください")
    return int(parsed)


def topic(value):
    if not re.fullmatch(r"/[A-Za-z_][A-Za-z0-9_/]*", str(value)):
        raise ValueError("topicは絶対名で指定してください")
    return str(value)


def read(path):
    return json.loads(Path(path).read_text())


def contained(value, root, directory=False):
    return resolve_under_root(str(value or ""), Path(root), label="section multihead asset", require_exists=True, require_directory=directory)


def new_output(value, root):
    path = resolve_under_root(_name(value, label="output name"), root)
    if path.exists():
        raise ValueError("出力先がすでに存在します。別の名前を指定してください")
    return path


def scan_maps(config):
    maps = []
    root = Path(config.map_root).resolve()
    if not root.is_dir():
        return maps
    for candidate in sorted(root.rglob("*.yaml")):
        if not (candidate.name.endswith("_hd_map.yaml") or candidate.name == "hd_map.yaml"):
            continue
        try:
            path = contained(candidate, root)
            document = load_yaml(path)
            geometry = contract(config).SectionMap(document)
            maps.append({"path":str(path), "name":str(path.relative_to(root)),
                         "sections":geometry.ids, "sha256":contract(config).map_digest(document)})
        except (OSError, ValueError, KeyError, TypeError):
            continue
    return maps


def snapshot(config):
    datasets, runs, analyses = roots(config)
    records = {"datasets":[], "runs":[], "analyses":[], "maps":scan_maps(config),
               "weights":scan_backbone_weights(config), "default_user":config.jetson_user,
               "default_host":(getattr(config, "jetson_ips", []) or [""])[0]}
    for key, root, filename in (("datasets",datasets,"section_dataset.json"), ("runs",runs,"metadata.json"), ("analyses",analyses,"report.json")):
        if not root.is_dir():
            continue
        for candidate in sorted(root.glob(f"*/{filename}"), reverse=True):
            try:
                path = contained(candidate, root)
                meta = read(path)
                record = {"path":str(path.parent), "name":path.parent.name, **meta}
                record.pop("map_document", None)
                record.pop("request", None)
                record.pop("training_request", None)
                if key == "runs":
                    record["has_onnx"] = (path.parent/"model.onnx").is_file()
                    metrics = path.parent/"metrics.json"
                    record["metrics"] = read(metrics) if metrics.is_file() else {}
                if key == "analyses":
                    preview = record.get("preview", [])
                    record["preview"] = preview[::max(1, len(preview)//400)]
                records[key].append(record)
            except (OSError, ValueError):
                continue
    return records


def bags(config, body):
    values = body.get("rosbags")
    if not isinstance(values, list) or not 1 <= len(values) <= 32:
        raise ValueError("rosbagは1〜32個選択してください")
    return list(dict.fromkeys(str(contained(value, config.record_root, True)) for value in values))


def input_options(body):
    clock = str(body.get("timestamp_source", "header"))
    if clock not in {"bag", "header"}:
        raise ValueError("timestamp_source must be bag or header")
    return {"image_topic":topic(body.get("image_topic", "/realsense/color/image_raw")),
            "control_topic":topic(body.get("control_topic", "/teleop/control_cmd")),
            "timestamp_source":clock,
            "max_control_dt_sec":number(body.get("max_control_dt_sec", .1), "操作時刻の許容差", .001, 2.)}


def job(config, action, request, resources):
    source = str(training_root(config)/"src")
    request_root = Path(config.state_dir)/"section_requests"
    request_root.mkdir(parents=True, exist_ok=True)
    request_path = request_root/f"{uuid.uuid4().hex}.json"
    request_path.write_text(json.dumps(request, ensure_ascii=False, allow_nan=False))
    command = [str(config.python_bin), "-m", "e2e_learning.cli.section_pipeline", action, "--request-file", str(request_path)]
    if request.get("mode") == "offline":
        setup = Path(config.ros2_ws)/"install"/"setup.bash"
        if not setup.is_file():
            raise ValueError("オフラインVSLAMにはビルド済みROS環境が必要です")
        script = "\n".join(["set -eo pipefail", f"source {shlex.quote(str(setup))}",
                              f"export PYTHONPATH={shlex.quote(source)}${{PYTHONPATH:+:${{PYTHONPATH}}}}",
                              shlex.join(command)])
        command = ["bash", "-lc", script]
    else:
        command = ["env", f"PYTHONPATH={source}{os.pathsep}{os.environ.get('PYTHONPATH', '')}", *command]
    output = request.get("output", request.get("run"))
    return PipelineTaskSpec(kind=f"section-multihead-{action}", title=f"Section Multihead {action}: {Path(output).name}",
                            command=command, cwd=str(training_root(config)),
                            artifacts=[{"name":"output", "path":str(output)}], resource_keys=resources)


def build_preprocess(config, body):
    data_root, _, _ = roots(config)
    path = contained(body.get("map"), config.map_root)
    document = load_yaml(path)
    geometry = contract(config).SectionMap(document)
    sections = body.get("sections")
    if not isinstance(sections, list) or not sections or not set(sections) <= set(geometry.ids):
        raise ValueError("選択した地図のsectionを1つ以上指定してください")
    mode = body.get("mode", "recorded")
    source = body.get("label_source", "auto")
    if mode not in {"recorded", "offline"} or source not in {"auto", "section", "pose"}:
        raise ValueError("不明なsection割り当て方式です")
    output = new_output(body.get("dataset_name"), data_root)
    request = {**input_options(body), "output":str(output), "rosbags":bags(config, body),
               "map":str(path), "map_document":document, "sections":list(dict.fromkeys(sections)),
               "mode":mode, "label_source":source,
               "section_topic":topic(body.get("section_topic", "/localization/current_section")),
               "pose_topic":topic(body.get("pose_topic", "/visual_slam/tracking/odometry")),
               "throttle":number(body.get("throttle", .2), "実行時throttle", 0., 1.),
               "max_label_age_sec":number(body.get("max_label_age_sec", .3), "ラベル有効時間", .01, 5.),
               "max_lane_distance_m":number(body.get("max_lane_distance_m", 1.), "lane許容距離", .01, 20.)}
    if body.get("filter_throttle") not in (None, ""):
        request["filter_throttle"] = number(body["filter_throttle"], "抽出throttle", 0., 1.)
        request["throttle_tolerance"] = number(body.get("throttle_tolerance", .015), "throttle許容差", 0., 1.)
    resources = [f"e2e-dataset:{output}", *[f"analysis-bag:{bag}" for bag in request["rosbags"]]]
    if mode == "offline":
        if request["timestamp_source"] != "header":
            raise ValueError("オフラインVSLAMは画像header時刻を使用してください")
        localization = contained(body.get("localization_map") or path.parent, config.map_root, True)
        if not list((localization/"cuvslam_map").glob("*.mdb")):
            raise ValueError("保存済みcuvslam_map/*.mdbを含む地図を指定してください")
        localization_mode = str(body.get("localization_mode", "origin"))
        if localization_mode not in {"origin", "vgl"}:
            raise ValueError("localization_mode must be origin or vgl")
        if localization_mode == "vgl" and not (localization/"cuvgl_map").is_dir():
            raise ValueError("VGLモードにはcuvgl_mapが必要です")
        local_hd = localization/f"{localization.name}_hd_map.yaml"
        if not local_hd.is_file():
            local_hd = localization/"hd_map.yaml"
        if not local_hd.is_file() or contract(config).map_digest(load_yaml(local_hd)) != contract(config).map_digest(document):
            raise ValueError("定位用mapのHD mapと抽出用section定義が一致しません")
        if body.get("camera_topic_config"):
            request["camera_topic_config"] = str(contained(body["camera_topic_config"], config.ros2_ws))
        request.update(localization_map=str(localization), localization_mode=localization_mode,
                       replay_rate=number(body.get("replay_rate", .5), "再生速度", .05, 2.),
                       ros_domain_id=getattr(config,"analysis_ros_domain_id",121))
        if request["ros_domain_id"] == int(os.environ.get("ROS_DOMAIN_ID", "0")):
            raise ValueError("オフライン再生には現在のROS domainと異なる解析用domainを設定してください")
        resources.append(f"analysis-ros-domain:{request['ros_domain_id']}")
        resources.append(f"map-dir:{localization}")
    return job(config, "preprocess", request, resources)


def build_train(config, body):
    data_root, run_root, _ = roots(config)
    raw_heads = body.get("heads")
    if not isinstance(raw_heads, list) or not 1 <= len(raw_heads) <= 32:
        raise ValueError("headは1〜32個指定してください")
    heads, metadata = [], []
    for raw in raw_heads:
        directory = contained(raw.get("dataset"), data_root, True)
        meta = read(directory/"section_dataset.json")
        metadata.append(meta)
        heads.append({"name":str(raw.get("name", "")), "dataset":str(directory), "sections":meta["sections"],
                      "generic":raw.get("generic") is True,
                      "throttle":number(raw.get("throttle", meta["recommended_throttle"]), "throttle", 0., 1.)})
    digest = contract(config).map_digest
    if len({m["map_sha256"] for m in metadata}) != 1 or any(m["map_sha256"] != digest(m["map_document"]) for m in metadata):
        raise ValueError("datasetの地図定義が一致しません")
    geometry = contract(config).SectionMap(metadata[0]["map_document"])
    contract(config).validate_heads(heads, geometry.ids)
    generic = next(h for h in heads if h["generic"])
    if set(generic["sections"]) != set(geometry.ids):
        raise ValueError("汎用headには全sectionのdatasetを割り当ててください")
    backbone = contained(body.get("backbone_weights"), training_root(config)/"weights")
    if not backbone.is_file() or backbone.suffix not in {".pt", ".pth"}:
        raise ValueError("DINOv3の重みファイルを選択してください")
    device = str(body.get("device", "cuda"))
    if not re.fullmatch(r"cpu|mps|cuda(?::[0-9]+)?", device):
        raise ValueError("device must be cpu, mps or cuda[:N]")
    output = new_output(body.get("run_name"), run_root)
    request = {"heads":heads, "backbone_weights":str(backbone), "device":device, "output":str(output),
               "epochs":integer(body.get("epochs", 30), "epochs", 1, 10000),
               "batch_size":integer(body.get("batch_size",32), "batch size", 1, 1024),
               "workers":integer(body.get("workers",4), "workers", 0, 32),
               "learning_rate":number(body.get("learning_rate",.001), "learning rate", 1e-8, 1.)}
    return job(config, "train", request, [f"section-run:{output}", *[f"e2e-dataset:{h['dataset']}" for h in heads]])


def build_export(config, body):
    run = contained(body.get("run"), roots(config)[1], True)
    if not (run/"checkpoints"/"best.pt").is_file():
        raise ValueError("学習済みcheckpointがありません")
    return job(config, "export", {"run":str(run)}, [f"section-run:{run}"])


def build_analyze(config, body):
    _, run_root, analysis_root = roots(config)
    run = contained(body.get("run"), run_root, True)
    metadata = read(run/"metadata.json")
    head = str(body.get("head", ""))
    if head not in metadata["output"]["head_order"] or not (run/"model.onnx").is_file():
        raise ValueError("export済みモデルとheadを選択してください")
    output = new_output(body.get("analysis_name"), analysis_root)
    request = {**input_options(body), "run":str(run), "head":head, "rosbags":bags(config, body), "output":str(output)}
    return job(config,"analyze",request,[f"section-run:{run}",f"section-analysis:{output}", *[f"analysis-bag:{b}" for b in request["rosbags"]]])


def build_deploy(config, body):
    run = contained(body.get("run"), roots(config)[1], True)
    metadata = read(run/"metadata.json")
    if metadata.get("model_kind") != "section_multihead" or not metadata.get("onnx_verified") or not (run/"model.onnx").is_file():
        raise ValueError("検証済みのMultihead ONNXを選択してください")
    user, host = str(body.get("user") or config.jetson_user), str(body.get("host") or "")
    target = validate_ssh_target(user, host)
    user, host = target.split("@", 1)
    remote = validate_remote_absolute_path(str(body.get("remote_root") or "/workspaces/ros2_ws/models/e2e/section_multihead"), label="remote model root")
    if not re.fullmatch(r"/[A-Za-z0-9_./-]+", remote):
        raise ValueError("転送先は英数字・underscore・ハイフン・スラッシュ・ドットのみ使用できます")
    if len(Path(remote).parts) < 4:
        raise ValueError("専用のモデル配備ディレクトリを指定してください")
    name = _name(body.get("deploy_name") or run.name, label="deploy name")
    command = ["bash", str(training_root(config)/"scripts"/"deploy_shared_vit_model.sh"), "--onnx",str(run/"model.onnx"),
               "--user",user,"--host",host,"--remote-root",remote,"--name",name]
    if body.get("build_engine", True):
        command.append("--build-engine")
    return PipelineTaskSpec(kind="section-multihead-deploy",title=f"Section Multihead deploy: {target}", command=command,
                            cwd=str(training_root(config)),artifacts=[{"name":"model", "path":str(run)}],
                            resource_keys=[f"section-run:{run}",f"section-deploy:{target}:{remote}/{name}"])
