#!/usr/bin/env python3
"""Generate the JetPilot topic graph from package README contracts."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET


TABLE_HEADER = ("Node", "Name", "Type", "QoS", "Description")
TOPIC_HEADING = re.compile(r"^### (Input|Output) topics\s*$")
HEADING = re.compile(r"^(#{1,6})\s+")


@dataclass(frozen=True, order=True)
class Endpoint:
    direction: str
    package: str
    node: str
    name: str
    type_name: str
    qos: str
    description: str
    readme: Path


class ContractError(ValueError):
    pass


def _cell(value: str) -> str:
    value = value.strip()
    if len(value) >= 2 and value[0] == "`" and value[-1] == "`":
        return value[1:-1].strip()
    return value


def _table_cells(line: str) -> tuple[str, ...]:
    if not line.lstrip().startswith("|"):
        return ()
    return tuple(_cell(part) for part in line.strip().strip("|").split("|"))


def _is_separator(cells: tuple[str, ...]) -> bool:
    return bool(cells) and all(re.fullmatch(r":?-{3,}:?", cell) for cell in cells)


def package_name(package_xml: Path) -> str:
    root = ET.parse(package_xml).getroot()
    name = root.findtext("name")
    if not name:
        raise ContractError(f"package name is missing: {package_xml}")
    return name.strip()


def parse_contract(readme: Path, package: str) -> list[Endpoint]:
    lines = readme.read_text(encoding="utf-8").splitlines()
    if "## Purpose" not in lines:
        raise ContractError(f"{readme}: missing '## Purpose'")
    if "## Inputs / Outputs" not in lines:
        raise ContractError(f"{readme}: missing '## Inputs / Outputs'")
    if "## Parameters" not in lines:
        raise ContractError(f"{readme}: missing '## Parameters'")
    if "## Assumptions / Known limits" not in lines:
        raise ContractError(f"{readme}: missing '## Assumptions / Known limits'")

    start = lines.index("## Inputs / Outputs") + 1
    end = len(lines)
    for index in range(start, len(lines)):
        if lines[index].startswith("## "):
            end = index
            break

    endpoints: list[Endpoint] = []
    index = start
    while index < end:
        match = TOPIC_HEADING.match(lines[index])
        if not match:
            index += 1
            continue
        direction = match.group(1).lower()
        index += 1
        while index < end and not lines[index].strip():
            index += 1
        if index >= end or not lines[index].lstrip().startswith("|"):
            continue

        header = _table_cells(lines[index])
        if header != TABLE_HEADER:
            raise ContractError(
                f"{readme}:{index + 1}: expected columns {TABLE_HEADER}, got {header}"
            )
        index += 1
        separator = _table_cells(lines[index]) if index < end else ()
        if not _is_separator(separator) or len(separator) != len(TABLE_HEADER):
            raise ContractError(f"{readme}:{index + 1}: invalid table separator")
        index += 1
        while index < end:
            if HEADING.match(lines[index]) or not lines[index].lstrip().startswith("|"):
                break
            cells = _table_cells(lines[index])
            if len(cells) != len(TABLE_HEADER):
                raise ContractError(f"{readme}:{index + 1}: expected five table cells")
            node, name, type_name, qos, description = cells
            if not all((node, name, type_name, qos, description)):
                raise ContractError(f"{readme}:{index + 1}: empty topic contract cell")
            endpoints.append(
                Endpoint(direction, package, node, name, type_name, qos, description, readme)
            )
            index += 1

    return endpoints


def discover(root: Path) -> tuple[list[Endpoint], list[Path]]:
    endpoints: list[Endpoint] = []
    readmes: list[Path] = []
    source_root = root / "ros2_ws" / "src"
    for package_xml in sorted(source_root.glob("**/package.xml")):
        package = package_name(package_xml)
        if not package.startswith("jetpilot_"):
            continue
        readme = package_xml.parent / "README.md"
        if not readme.is_file():
            raise ContractError(f"{package}: README.md is missing")
        readmes.append(readme)
        endpoints.extend(parse_contract(readme, package))
    return sorted(endpoints), readmes


def validate(endpoints: list[Endpoint]) -> None:
    topic_types: dict[str, set[str]] = {}
    by_topic: dict[str, list[Endpoint]] = {}
    duplicate_keys: set[tuple[str, str, str, str]] = set()
    for item in endpoints:
        key = (item.direction, item.package, item.node, item.name)
        if key in duplicate_keys:
            raise ContractError(
                f"duplicate {item.direction} endpoint: {item.package}/{item.node} {item.name}"
            )
        duplicate_keys.add(key)
        if item.name.startswith("/"):
            topic_types.setdefault(item.name, set()).add(item.type_name)
            by_topic.setdefault(item.name, []).append(item)
    conflicts = {name: types for name, types in topic_types.items() if len(types) > 1}
    if conflicts:
        details = ", ".join(
            f"{name}: {', '.join(sorted(types))}" for name, types in sorted(conflicts.items())
        )
        raise ContractError(f"topic type mismatch: {details}")

    qos_errors: list[str] = []
    for name, members in sorted(by_topic.items()):
        publishers = [item for item in members if item.direction == "output"]
        subscribers = [item for item in members if item.direction == "input"]
        for publisher in publishers:
            pub_qos = {part.strip().lower() for part in publisher.qos.split("/")}
            for subscriber in subscribers:
                sub_qos = {part.strip().lower() for part in subscriber.qos.split("/")}
                if "best effort" in pub_qos and "reliable" in sub_qos:
                    qos_errors.append(
                        f"{name}: best-effort publisher {publisher.package}/{publisher.node} "
                        f"cannot satisfy reliable subscriber {subscriber.package}/{subscriber.node}"
                    )
                if "volatile" in pub_qos and "transient local" in sub_qos:
                    qos_errors.append(
                        f"{name}: volatile publisher {publisher.package}/{publisher.node} "
                        f"cannot satisfy transient-local subscriber "
                        f"{subscriber.package}/{subscriber.node}"
                    )
    if qos_errors:
        raise ContractError("QoS incompatibility: " + "; ".join(qos_errors))


def _mermaid_id(prefix: str, value: str) -> str:
    digest = hashlib.sha1(value.encode("utf-8"), usedforsecurity=False).hexdigest()[:10]
    return f"{prefix}_{digest}"


def _escape(value: str) -> str:
    return value.replace("\\", "\\\\").replace('"', "&quot;")


def render(root: Path, endpoints: list[Endpoint], readmes: list[Path]) -> str:
    absolute = [item for item in endpoints if item.name.startswith("/")]
    topics = sorted({item.name for item in absolute})
    nodes = sorted({(item.package, item.node) for item in absolute})
    lines = [
        "# JetPilot topic graph",
        "",
        "この文書は各 `jetpilot_*` package の `README.md` にある "
        "`Inputs / Outputs` 表から生成されています。手動で編集しないでください。",
        "",
        "図は全機能を重ねた静的なsupersetです。同じtopicを使う排他的なlaunch構成も同時に表示されます。"
        "実行時のnamespace、任意remap、外部package内部のinterfaceは反映されません。",
        "",
        "## System graph",
        "",
        "```mermaid",
        "flowchart LR",
        "  classDef topic fill:#fff4cc,stroke:#9a7600,color:#222;",
        "  classDef node fill:#e8f1ff,stroke:#3167a8,color:#111;",
    ]
    for package, node in nodes:
        node_id = _mermaid_id("node", f"{package}/{node}")
        label = _escape(f"{package}<br/>{node}")
        lines.append(f'  {node_id}["{label}"]:::node')
    for topic in topics:
        topic_id = _mermaid_id("topic", topic)
        lines.append(f'  {topic_id}(["{_escape(topic)}"]):::topic')
    for item in absolute:
        node_id = _mermaid_id("node", f"{item.package}/{item.node}")
        topic_id = _mermaid_id("topic", item.name)
        if item.direction == "output":
            lines.append(f"  {node_id} --> {topic_id}")
        else:
            lines.append(f"  {topic_id} --> {node_id}")
    lines.extend(["```", "", "## Topic inventory", ""])
    lines.append("| Topic | Type | Publishers | Subscribers |")
    lines.append("| --- | --- | --- | --- |")
    for topic in topics:
        members = [item for item in absolute if item.name == topic]
        type_name = members[0].type_name
        publishers = sorted(
            f"`{item.package}/{item.node}`" for item in members if item.direction == "output"
        )
        subscribers = sorted(
            f"`{item.package}/{item.node}`" for item in members if item.direction == "input"
        )
        lines.append(
            f"| `{topic}` | `{type_name}` | {', '.join(publishers) or '外部'} | "
            f"{', '.join(subscribers) or '外部'} |"
        )
    relative = [item for item in endpoints if not item.name.startswith("/")]
    if relative:
        lines.extend(
            [
                "",
                "## Relative or configurable topic names",
                "",
                "絶対名でないため、system graphでは自動結線していないinterfaceです。",
                "",
                "| Direction | Package / Node | Name | Type |",
                "| --- | --- | --- | --- |",
            ]
        )
        for item in relative:
            lines.append(
                f"| {item.direction} | `{item.package}/{item.node}` | `{item.name}` | "
                f"`{item.type_name}` |"
            )
    lines.extend(
        [
            "",
            "## Source",
            "",
            f"- Packages: {len(readmes)}",
            f"- Topic endpoints: {len(endpoints)}",
            "- Generator: `scripts/generate_topic_graph.py`",
            "- Contract: `docs/ros_package_readme_guideline.md`",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, help="repository root")
    parser.add_argument("--output", type=Path, help="output Markdown path")
    parser.add_argument("--check", action="store_true", help="fail if output is stale")
    parser.add_argument("--validate-only", action="store_true", help="validate without writing")
    args = parser.parse_args()

    root = (args.root or Path(__file__).resolve().parents[1]).resolve()
    output = (args.output or root / "docs" / "topic_graph.md").resolve()
    try:
        endpoints, readmes = discover(root)
        validate(endpoints)
        generated = render(root, endpoints, readmes)
    except (ContractError, ET.ParseError, OSError) as error:
        print(f"topic graph error: {error}", file=sys.stderr)
        return 2

    if args.validate_only:
        print(f"validated {len(readmes)} package README files and {len(endpoints)} endpoints")
        return 0
    if args.check:
        current = output.read_text(encoding="utf-8") if output.is_file() else ""
        if current != generated:
            print(f"topic graph is stale: run {Path(__file__).name}", file=sys.stderr)
            return 1
        print(f"topic graph is current: {output}")
        return 0
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(generated, encoding="utf-8")
    print(f"generated {output} from {len(readmes)} package README files")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
