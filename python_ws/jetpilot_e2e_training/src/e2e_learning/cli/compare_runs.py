import csv
import json
from pathlib import Path

import hydra
from omegaconf import DictConfig


FIELDS = [
    "run_name",
    "task",
    "model",
    "backbone",
    "temporal",
    "use_imu",
    "sequence_length",
    "data_fraction",
    "val_loss",
    "steering_mae",
    "throttle_mae",
    "steering_rmse",
    "throttle_rmse",
    "trajectory_ade_m",
    "trajectory_fde_m",
    "trajectory_lateral_mae_m",
]


def summarize(root: Path) -> list[dict[str, str]]:
    rows = []
    for metrics_path in sorted(root.glob("**/metrics.json")):
        payload = json.loads(metrics_path.read_text())
        best = payload.get("best", {})
        architecture = payload.get("architecture", {})
        rows.append(
            {
                "run_name": payload.get("run_name", metrics_path.parent.name),
                "task": str(payload.get("task", "control")),
                "model": payload.get("model", ""),
                "backbone": str(architecture.get("backbone", "")),
                "temporal": str(architecture.get("temporal", "")),
                "use_imu": str(architecture.get("use_imu", "")),
                "sequence_length": str(architecture.get("sequence_length", "")),
                "data_fraction": str(payload.get("data_fraction", "")),
                "val_loss": str(best.get("loss", "")),
                "steering_mae": str(best.get("steering_mae", "")),
                "throttle_mae": str(best.get("throttle_mae", "")),
                "steering_rmse": str(best.get("steering_rmse", "")),
                "throttle_rmse": str(best.get("throttle_rmse", "")),
                "trajectory_ade_m": str(best.get("trajectory_ade_m", "")),
                "trajectory_fde_m": str(best.get("trajectory_fde_m", "")),
                "trajectory_lateral_mae_m": str(best.get("trajectory_lateral_mae_m", "")),
            }
        )
    return rows


@hydra.main(config_path="../conf", config_name="config", version_base=None)
def main(cfg: DictConfig) -> None:
    root = Path(str(cfg.compare.root)).expanduser()
    rows = summarize(root)
    if not rows:
        raise RuntimeError(f"No metrics.json files found under {root}")
    csv_path = root / "summary.csv"
    with csv_path.open("w", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    md_path = root / "summary.md"
    md_lines = ["| " + " | ".join(FIELDS) + " |", "|" + "|".join(["---"] * len(FIELDS)) + "|"]
    for row in rows:
        md_lines.append("| " + " | ".join(row[field] for field in FIELDS) + " |")
    md_path.write_text("\n".join(md_lines) + "\n")
    print(f"Wrote {csv_path}")
    print(f"Wrote {md_path}")


if __name__ == "__main__":
    main()
