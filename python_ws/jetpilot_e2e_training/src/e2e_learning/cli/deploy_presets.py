import argparse
import json
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    payload = json.loads(Path(args.config).read_text())
    default_id = payload.get("default", "")
    for item in payload.get("presets", []):
        print(
            "\t".join(
                [
                    item["id"],
                    item.get("label", item["id"]),
                    item.get("description", ""),
                    item["model_name"],
                    item.get("model_kind", ""),
                    item.get("modality", ""),
                    item.get("metadata_filename", "metadata.json"),
                    "1" if item["id"] == default_id else "0",
                ]
            )
        )


if __name__ == "__main__":
    main()
