#!/usr/bin/env python3
"""Rewrite message header.frame_id for selected topics in a ROS 2 bag."""

from __future__ import annotations

import argparse
from pathlib import Path

from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import ConverterOptions, SequentialReader, SequentialWriter, StorageOptions
from rosidl_runtime_py.utilities import get_message


def load_yaml(path: Path) -> dict:
    try:
        import yaml  # type: ignore

        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except Exception as exc:
        raise RuntimeError(f"Could not parse YAML: {path}") from exc


def bag_uri(path: Path) -> Path:
    if path.is_file() and (path.parent / "metadata.yaml").exists():
        return path.parent
    return path


def storage_id(path: Path) -> str:
    uri = bag_uri(path)
    if uri.is_file():
        if uri.suffix == ".mcap":
            return "mcap"
        if uri.suffix == ".db3":
            return "sqlite3"

    metadata_path = uri / "metadata.yaml"
    if metadata_path.exists():
        metadata = load_yaml(metadata_path)
        info = metadata.get("rosbag2_bagfile_information", {})
        value = info.get("storage_identifier")
        if value:
            return str(value)

    if uri.is_dir() and list(uri.glob("*.mcap")):
        return "mcap"
    return "sqlite3"


def open_reader(path: Path) -> tuple[SequentialReader, dict[str, str], list[object]]:
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_uri(path)), storage_id=storage_id(path)),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_metadata = list(reader.get_all_topics_and_types())
    topic_types = {topic.name: topic.type for topic in topic_metadata}
    return reader, topic_types, topic_metadata


def run(args: argparse.Namespace) -> int:
    input_bag = Path(args.input_bag).expanduser().resolve()
    output_bag = Path(args.output_bag).expanduser().resolve()
    rewrite_topics = args.topic or ["/camera/right/camera_info"]

    if output_bag.exists() and not args.overwrite:
        raise RuntimeError(f"Output bag already exists: {output_bag}")

    if output_bag.exists() and args.overwrite:
        # Avoid recursive deletion surprises: only allow replacing bag directories
        # that already look like rosbag2 outputs.
        if not (output_bag / "metadata.yaml").exists():
            raise RuntimeError(f"Refusing to overwrite non-bag directory: {output_bag}")
        import shutil

        shutil.rmtree(output_bag)

    reader, topic_types, topic_metadata = open_reader(input_bag)
    missing_topics = [topic for topic in rewrite_topics if topic not in topic_types]
    if missing_topics:
        raise RuntimeError(f"Topic(s) not found in input bag: {', '.join(missing_topics)}")

    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=str(output_bag), storage_id=args.storage_id),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    for topic in topic_metadata:
        writer.create_topic(topic)

    message_types = {topic: get_message(topic_types[topic]) for topic in rewrite_topics}
    rewritten = 0
    total = 0

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        total += 1
        if topic in message_types:
            msg = deserialize_message(data, message_types[topic])
            header = getattr(msg, "header", None)
            if header is None:
                raise RuntimeError(f"Topic has no header field: {topic}")
            if msg.header.frame_id != args.frame_id:
                msg.header.frame_id = args.frame_id
                rewritten += 1
            data = serialize_message(msg)
        writer.write(topic, data, timestamp)

    print(f"Input bag:  {input_bag}")
    print(f"Output bag: {output_bag}")
    print(f"Topic(s):   {', '.join(rewrite_topics)}")
    print(f"Frame id:   {args.frame_id}")
    print(f"Messages:   {total}")
    print(f"Rewritten:  {rewritten}")
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-bag", required=True, help="Input rosbag directory or storage file")
    parser.add_argument("--output-bag", required=True, help="Output rosbag directory")
    parser.add_argument("--topic", action="append", default=None, help="Topic to rewrite. Can be passed multiple times.")
    parser.add_argument("--frame-id", default="realsense_infra2_optical_frame")
    parser.add_argument("--storage-id", default="mcap")
    parser.add_argument("--overwrite", action="store_true")
    return parser


def main() -> None:
    raise SystemExit(run(build_arg_parser().parse_args()))


if __name__ == "__main__":
    main()
