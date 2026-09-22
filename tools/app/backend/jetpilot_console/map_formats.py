"""Dependency-free YAML/JSON reader for map and worker metadata."""
from __future__ import annotations
import json
from ast import literal_eval
from pathlib import Path
from typing import Any

def _strip_comment(line: str) -> str:
    in_quote = ""
    for index, char in enumerate(line):
        if char in ("'", '"'):
            if in_quote == char:
                in_quote = ""
            elif not in_quote:
                in_quote = char
        if char == "#" and not in_quote:
            return line[:index]
    return line


def _split_inline_yaml_values(value: str) -> list[str]:
    values: list[str] = []
    current: list[str] = []
    quote = ""
    escaped = False
    depth = 0
    for char in value:
        if escaped:
            current.append(char)
            escaped = False
            continue
        if char == "\\" and quote:
            current.append(char)
            escaped = True
            continue
        if char in {"'", '"'}:
            if quote == char:
                quote = ""
            elif not quote:
                quote = char
            current.append(char)
            continue
        if not quote:
            if char in "[({":
                depth += 1
            elif char in "])}" and depth > 0:
                depth -= 1
            elif char == "," and depth == 0:
                values.append("".join(current).strip())
                current = []
                continue
        current.append(char)
    values.append("".join(current).strip())
    return values


def _clean_scalar(value: str) -> Any:
    text = value.strip()
    if text == "":
        return ""
    lowered = text.lower()
    if lowered in {"null", "none", "~"}:
        return None
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    if len(text) >= 2 and text[0] == text[-1] and text[0] in {"'", '"'}:
        if text[0] == '"':
            try:
                return json.loads(text)
            except (json.JSONDecodeError, TypeError):
                pass
        return text[1:-1]
    if text.startswith("[") and text.endswith("]"):
        try:
            return literal_eval(text)
        except (SyntaxError, ValueError):
            inner = text[1:-1].strip()
            if not inner:
                return []
            return [_clean_scalar(item) for item in _split_inline_yaml_values(inner)]
    try:
        if any(char in text for char in ".eE"):
            return float(text)
        return int(text)
    except ValueError:
        return text


def _yaml_lines(path: Path) -> list[tuple[int, str]]:
    lines = []
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        cleaned = _strip_comment(raw).rstrip()
        if not cleaned.strip():
            continue
        indent = len(cleaned) - len(cleaned.lstrip(" "))
        lines.append((indent, cleaned.strip()))
    return lines


def _next_indent(lines: list[tuple[int, str]], index: int, fallback: int) -> int:
    if index < len(lines):
        return lines[index][0]
    return fallback


def _parse_yaml_block(lines: list[tuple[int, str]], index: int, indent: int) -> tuple[Any, int]:
    if index >= len(lines):
        return {}, index
    if lines[index][0] < indent:
        return {}, index
    if lines[index][1].startswith("- "):
        return _parse_yaml_list(lines, index, lines[index][0])
    return _parse_yaml_map(lines, index, indent)


def _parse_yaml_map(lines: list[tuple[int, str]], index: int, indent: int) -> tuple[dict[str, Any], int]:
    data: dict[str, Any] = {}
    while index < len(lines):
        line_indent, text = lines[index]
        if line_indent < indent:
            break
        if line_indent > indent:
            index += 1
            continue
        if text.startswith("- ") or ":" not in text:
            break
        key, value = text.split(":", 1)
        key = key.strip()
        value = value.strip()
        index += 1
        if value:
            data[key] = _clean_scalar(value)
        else:
            child_indent = _next_indent(lines, index, line_indent + 2)
            data[key], index = _parse_yaml_block(lines, index, child_indent)
    return data, index


def _parse_yaml_list(lines: list[tuple[int, str]], index: int, indent: int) -> tuple[list[Any], int]:
    items: list[Any] = []
    while index < len(lines):
        line_indent, text = lines[index]
        if line_indent < indent:
            break
        if line_indent != indent or not text.startswith("- "):
            break

        item_text = text[2:].strip()
        index += 1

        if item_text.startswith("- "):
            nested = [_clean_scalar(item_text[2:].strip())]
            while index < len(lines) and lines[index][0] > indent and lines[index][1].startswith("- "):
                nested.append(_clean_scalar(lines[index][1][2:].strip()))
                index += 1
            items.append(nested)
            continue

        if not item_text:
            child_indent = _next_indent(lines, index, indent + 2)
            child, index = _parse_yaml_block(lines, index, child_indent)
            items.append(child)
            continue

        if ":" in item_text and not item_text.startswith("["):
            item: dict[str, Any] = {}
            key, value = item_text.split(":", 1)
            item[key.strip()] = _clean_scalar(value.strip()) if value.strip() else {}
            while index < len(lines):
                next_indent, next_text = lines[index]
                if next_indent <= indent:
                    break
                if next_text.startswith("- ") and next_indent == indent:
                    break
                if ":" not in next_text:
                    index += 1
                    continue
                child_key, child_value = next_text.split(":", 1)
                child_key = child_key.strip()
                child_value = child_value.strip()
                index += 1
                if child_value:
                    item[child_key] = _clean_scalar(child_value)
                else:
                    child_indent = _next_indent(lines, index, next_indent + 2)
                    item[child_key], index = _parse_yaml_block(lines, index, child_indent)
            items.append(item)
            continue

        items.append(_clean_scalar(item_text))
    return items, index


def load_yaml(path: Path) -> dict[str, Any]:
    # JSON is a valid subset of YAML. Some dependency-light workers emit their
    # metadata.yaml in this form, so handle it before the built-in YAML parser.
    raw = path.read_text(encoding="utf-8")
    if raw.lstrip().startswith(("{", "[")):
        try:
            json_data = json.loads(raw)
        except json.JSONDecodeError:
            pass
        else:
            return json_data if isinstance(json_data, dict) else {}
    lines = _yaml_lines(path)
    data, _ = _parse_yaml_block(lines, 0, lines[0][0] if lines else 0)
    return data if isinstance(data, dict) else {}


