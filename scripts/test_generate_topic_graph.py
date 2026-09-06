#!/usr/bin/env python3

from pathlib import Path
import tempfile
import unittest

import generate_topic_graph as graph


CONTRACT = """# example

## Purpose

Example.

## Inputs / Outputs

### Input topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `consumer` | `/sample` | `std_msgs/msg/String` | Reliable / Volatile | input |

### Output topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `producer` | `/sample` | `std_msgs/msg/String` | Reliable / Volatile | output |

## Parameters

None.

## Assumptions / Known limits

None.
"""


class TopicGraphTest(unittest.TestCase):
    def test_parse_and_render(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            readme = root / "README.md"
            readme.write_text(CONTRACT, encoding="utf-8")
            endpoints = graph.parse_contract(readme, "example")
            graph.validate(endpoints)
            rendered = graph.render(root, endpoints, [readme])
            self.assertEqual(len(endpoints), 2)
            self.assertIn("/sample", rendered)
            self.assertIn("example/producer", rendered)
            self.assertIn("example/consumer", rendered)

    def test_type_mismatch_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            readme = Path(directory) / "README.md"
            readme.write_text(CONTRACT, encoding="utf-8")
            endpoints = graph.parse_contract(readme, "example")
            bad = graph.Endpoint(
                "input",
                "other",
                "consumer",
                "/sample",
                "std_msgs/msg/Bool",
                "Reliable / Volatile",
                "bad",
                readme,
            )
            with self.assertRaises(graph.ContractError):
                graph.validate(endpoints + [bad])

    def test_qos_mismatch_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            readme = Path(directory) / "README.md"
            readme.write_text(CONTRACT, encoding="utf-8")
            endpoints = graph.parse_contract(readme, "example")
            publisher = next(item for item in endpoints if item.direction == "output")
            incompatible = graph.Endpoint(
                publisher.direction,
                publisher.package,
                publisher.node,
                publisher.name,
                publisher.type_name,
                "Best Effort / Volatile",
                publisher.description,
                publisher.readme,
            )
            compatible_input = next(item for item in endpoints if item.direction == "input")
            with self.assertRaises(graph.ContractError):
                graph.validate([incompatible, compatible_input])

    def test_required_sections_are_enforced(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            readme = Path(directory) / "README.md"
            readme.write_text("# incomplete\n", encoding="utf-8")
            with self.assertRaises(graph.ContractError):
                graph.parse_contract(readme, "example")


if __name__ == "__main__":
    unittest.main()
