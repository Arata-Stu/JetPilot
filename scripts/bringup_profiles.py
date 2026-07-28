#!/usr/bin/env python3
"""Validate and resolve JetPilot bringup profile manifests."""

from __future__ import annotations

import argparse
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


SCHEMA_VERSION = 1
KINDS = ("vehicle", "sensor_kit")
ID_PATTERN = re.compile(r"^[a-z0-9][a-z0-9-]*$")
ALIAS_PATTERN = re.compile(r"^[a-z0-9][a-z0-9_-]*$")
ARGUMENT_PATTERN = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
PACKAGE_PATTERN = re.compile(r"^[a-z][a-z0-9_]*$")
TOP_LEVEL_FIELDS = {
    "schema_version",
    "kind",
    "id",
    "label",
    "order",
    "aliases",
    "launch",
    "driver_param",
    "arguments",
    "rtp_topics",
}


class ProfileError(ValueError):
    pass


def fail(message: str) -> None:
    raise ProfileError(message)


def check_text(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value:
        fail(f"{field} must be a non-empty string")
    if "\t" in value or "\n" in value or "\r" in value:
        fail(f"{field} must not contain tabs or newlines")
    return value


def check_id(value: Any, field: str) -> str:
    text = check_text(value, field)
    if not ID_PATTERN.fullmatch(text):
        fail(f"{field} must match {ID_PATTERN.pattern}: {text}")
    return text


def check_alias(value: Any, field: str) -> str:
    text = check_text(value, field)
    if not ALIAS_PATTERN.fullmatch(text):
        fail(f"{field} must match {ALIAS_PATTERN.pattern}: {text}")
    return text


def check_relative_path(value: Any, field: str) -> str:
    text = check_text(value, field)
    path = Path(text)
    if path.is_absolute() or ".." in path.parts:
        fail(f"{field} must be a safe relative path: {text}")
    return text


def scalar_text(value: Any, field: str) -> str:
    if isinstance(value, bool):
        return str(value).lower()
    if isinstance(value, (str, int, float)) and not isinstance(value, complex):
        return check_text(str(value), field)
    fail(f"{field} must be a string, number, or boolean")
    raise AssertionError("unreachable")


def check_argument_name(kind: str, name: str, path: Path) -> None:
    if kind == "vehicle":
        allowed = (
            name == "vehicle_control_topic"
            or name.startswith("vehicle_description_")
            or name.startswith("publish_vehicle_")
        )
    else:
        allowed = name.startswith("sensor_kit_") and name not in {
            "sensor_kit_interface_pkg",
            "sensor_kit_interface_launch",
        }
    if not allowed:
        fail(f"{path}: argument is not allowed for {kind} profiles: {name}")


def validate_profile(payload: Any, path: Path, expected_kind: str) -> dict[str, Any]:
    if not isinstance(payload, dict):
        fail(f"{path}: profile must be a JSON object")

    unknown = set(payload) - TOP_LEVEL_FIELDS
    if unknown:
        fail(f"{path}: unknown fields: {', '.join(sorted(unknown))}")

    if payload.get("schema_version") != SCHEMA_VERSION:
        fail(f"{path}: schema_version must be {SCHEMA_VERSION}")

    kind = check_text(payload.get("kind"), f"{path}: kind")
    if kind != expected_kind:
        fail(f"{path}: kind must be {expected_kind}")

    profile_id = check_id(payload.get("id"), f"{path}: id")
    if path.stem != profile_id:
        fail(f"{path}: filename must be {profile_id}.json")

    label = check_text(payload.get("label"), f"{path}: label")
    order = payload.get("order", 100)
    if not isinstance(order, int) or isinstance(order, bool):
        fail(f"{path}: order must be an integer")

    aliases = payload.get("aliases", [])
    if not isinstance(aliases, list):
        fail(f"{path}: aliases must be an array")
    checked_aliases = [check_alias(alias, f"{path}: aliases") for alias in aliases]
    if profile_id in checked_aliases or len(checked_aliases) != len(set(checked_aliases)):
        fail(f"{path}: aliases must be unique and must not repeat id")

    launch = payload.get("launch")
    if not isinstance(launch, dict):
        fail(f"{path}: launch must be an object")
    unknown_launch = set(launch) - {"package", "file"}
    if unknown_launch:
        fail(f"{path}: unknown launch fields: {', '.join(sorted(unknown_launch))}")
    launch_package = check_text(launch.get("package"), f"{path}: launch.package")
    if not PACKAGE_PATTERN.fullmatch(launch_package):
        fail(f"{path}: invalid ROS package name: {launch_package}")
    launch_file = check_relative_path(launch.get("file"), f"{path}: launch.file")
    if not launch_file.startswith("launch/") or not launch_file.endswith(
        (".launch.py", ".launch.xml")
    ):
        fail(f"{path}: launch.file must be launch/*.launch.py or launch/*.launch.xml")

    arguments = payload.get("arguments", {})
    if not isinstance(arguments, dict):
        fail(f"{path}: arguments must be an object")
    checked_arguments: dict[str, str] = {}
    for name, value in arguments.items():
        if not ARGUMENT_PATTERN.fullmatch(name):
            fail(f"{path}: invalid launch argument name: {name}")
        check_argument_name(kind, name, path)
        checked_arguments[name] = scalar_text(value, f"{path}: arguments.{name}")

    driver_param = payload.get("driver_param")
    checked_driver_param: dict[str, str] | None = None
    if kind == "vehicle":
        if not isinstance(driver_param, dict):
            fail(f"{path}: vehicle profile requires driver_param")
        unknown_param = set(driver_param) - {"package", "path", "workspace_override"}
        if unknown_param:
            fail(
                f"{path}: unknown driver_param fields: "
                f"{', '.join(sorted(unknown_param))}"
            )
        param_package = check_text(
            driver_param.get("package"), f"{path}: driver_param.package"
        )
        if not PACKAGE_PATTERN.fullmatch(param_package):
            fail(f"{path}: invalid driver_param package name: {param_package}")
        checked_driver_param = {
            "package": param_package,
            "path": check_relative_path(
                driver_param.get("path"), f"{path}: driver_param.path"
            ),
        }
        workspace_override = driver_param.get("workspace_override")
        if workspace_override is not None:
            checked_driver_param["workspace_override"] = check_relative_path(
                workspace_override, f"{path}: driver_param.workspace_override"
            )
    elif driver_param is not None:
        fail(f"{path}: sensor_kit profile must not define driver_param")

    rtp_topics = payload.get("rtp_topics", [])
    if not isinstance(rtp_topics, list):
        fail(f"{path}: rtp_topics must be an array")
    checked_topics = []
    for topic in rtp_topics:
        topic_text = check_text(topic, f"{path}: rtp_topics")
        if not topic_text.startswith("/"):
            fail(f"{path}: RTP topic must be absolute: {topic_text}")
        checked_topics.append(topic_text)
    if len(checked_topics) != len(set(checked_topics)):
        fail(f"{path}: rtp_topics must be unique")
    if kind == "vehicle" and checked_topics:
        fail(f"{path}: vehicle profile must not define rtp_topics")

    return {
        "path": path,
        "kind": kind,
        "id": profile_id,
        "label": label,
        "order": order,
        "aliases": checked_aliases,
        "launch_package": launch_package,
        "launch_file": launch_file,
        "driver_param": checked_driver_param,
        "arguments": checked_arguments,
        "rtp_topics": checked_topics,
    }


def load_profiles(root: Path, kind: str) -> list[dict[str, Any]]:
    if kind not in KINDS:
        fail(f"unknown profile kind: {kind}")
    directory = root / kind
    if not directory.is_dir():
        fail(f"profile directory was not found: {directory}")

    profiles = []
    for path in sorted(directory.glob("*.json")):
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            fail(f"{path}: could not read JSON: {error}")
        profiles.append(validate_profile(payload, path, kind))
    if not profiles:
        fail(f"no {kind} profiles were found in {directory}")

    names: dict[str, Path] = {}
    for profile in profiles:
        for name in [profile["id"], *profile["aliases"]]:
            if name in names:
                fail(
                    f"duplicate {kind} profile id/alias '{name}': "
                    f"{names[name]} and {profile['path']}"
                )
            names[name] = profile["path"]
    return sorted(profiles, key=lambda item: (item["order"], item["label"], item["id"]))


def find_source_package(package: str, roots: list[Path]) -> Path | None:
    seen: set[Path] = set()
    for root in roots:
        try:
            resolved_root = root.resolve()
        except OSError:
            continue
        if resolved_root in seen or not resolved_root.is_dir():
            continue
        seen.add(resolved_root)
        for package_xml in resolved_root.rglob("package.xml"):
            try:
                name = ET.parse(package_xml).getroot().findtext("name")
            except (OSError, ET.ParseError):
                continue
            if name == package:
                return package_xml.parent
    return None


def resolve_driver_param(
    spec: dict[str, str], project_root: Path, ros2_ws: Path
) -> str:
    candidates: list[Path] = []
    override = spec.get("workspace_override")
    if override:
        candidates.extend(
            [
                ros2_ws / "joy_profiles" / override,
                project_root / "ros2_ws" / "joy_profiles" / override,
            ]
        )

    source_package = find_source_package(
        spec["package"],
        [ros2_ws / "src", project_root / "ros2_ws" / "src"],
    )
    if source_package is not None:
        candidates.append(source_package / spec["path"])
    installed = (
        ros2_ws
        / "install"
        / spec["package"]
        / "share"
        / spec["package"]
        / spec["path"]
    )
    candidates.append(installed)

    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return str(installed)


def emit(key: str, value: str) -> None:
    print(f"{key}\t{value}")


def command_list(args: argparse.Namespace) -> None:
    for profile in load_profiles(args.root, args.kind):
        emit(profile["id"], profile["label"])


def command_resolve(args: argparse.Namespace) -> None:
    profiles = load_profiles(args.root, args.kind)
    profile = next(
        (
            item
            for item in profiles
            if args.id == item["id"] or args.id in item["aliases"]
        ),
        None,
    )
    if profile is None:
        available = ", ".join(item["id"] for item in profiles)
        fail(f"unknown {args.kind} profile '{args.id}' (available: {available})")

    emit("id", profile["id"])
    emit("label", profile["label"])
    emit("launch_package", profile["launch_package"])
    emit("launch_file", profile["launch_file"])
    if profile["driver_param"] is not None:
        emit(
            "driver_param",
            resolve_driver_param(
                profile["driver_param"], args.project_root, args.ros2_ws
            ),
        )
    for name, value in profile["arguments"].items():
        emit(f"argument:{name}", value)
    for topic in profile["rtp_topics"]:
        emit("rtp_topic", topic)


def command_validate(args: argparse.Namespace) -> None:
    count = 0
    for kind in KINDS:
        profiles = load_profiles(args.root, kind)
        count += len(profiles)
        print(f"{kind}: {len(profiles)} profile(s)")
    print(f"validated: {count} profile(s)")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    subparsers = parser.add_subparsers(dest="command", required=True)

    list_parser = subparsers.add_parser("list")
    list_parser.add_argument("--kind", choices=KINDS, required=True)
    list_parser.set_defaults(handler=command_list)

    resolve_parser = subparsers.add_parser("resolve")
    resolve_parser.add_argument("--kind", choices=KINDS, required=True)
    resolve_parser.add_argument("--id", required=True)
    resolve_parser.add_argument("--project-root", type=Path, required=True)
    resolve_parser.add_argument("--ros2-ws", type=Path, required=True)
    resolve_parser.set_defaults(handler=command_resolve)

    validate_parser = subparsers.add_parser("validate")
    validate_parser.set_defaults(handler=command_validate)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        args.handler(args)
    except ProfileError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
