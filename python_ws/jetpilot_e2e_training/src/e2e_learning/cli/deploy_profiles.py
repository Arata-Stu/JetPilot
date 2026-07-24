import argparse
import json
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    payload = json.loads(Path(args.config).read_text())
    default_id = payload.get("default", "")
    for item in payload.get("profiles", []):
        print(
            "\t".join(
                [
                    item["id"],
                    item.get("label", item["id"]),
                    item.get("description", ""),
                    item["user"],
                    item["host"],
                    item["remote_root"],
                    "1" if item["id"] == default_id else "0",
                ]
            )
        )


if __name__ == "__main__":
    main()
