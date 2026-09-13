from pathlib import Path

import hydra
from omegaconf import DictConfig, OmegaConf

from e2e_learning.data.rosbag_extractor import ExtractConfig, extract_dataset


@hydra.main(config_path="../conf", config_name="config", version_base=None)
def main(cfg: DictConfig) -> None:
    data = cfg.data
    extract_cfg = ExtractConfig(
        bag_path=Path(data.bag_path).expanduser(),
        output_dir=Path(data.output_dir).expanduser(),
        image_topic=str(data.image_topic),
        control_topic=str(data.control_topic),
        input_width=int(data.input_width),
        input_height=int(data.input_height),
        max_control_dt_sec=float(data.max_control_dt_sec),
        timestamp_source=str(data.timestamp_source),
        task=str(data.task),
        odometry_topic=str(data.odometry_topic),
        imu_topic=str(data.imu_topic),
        max_odometry_dt_sec=float(data.max_odometry_dt_sec),
        trajectory_horizon_sec=float(data.trajectory_horizon_sec),
        trajectory_points=int(data.trajectory_points),
        trajectory_scale_m=float(data.trajectory_scale_m),
        imu_window_sec=float(data.imu_window_sec),
        imu_samples=int(data.imu_samples),
        image_extension=str(data.image_extension),
        jpeg_quality=int(data.jpeg_quality),
        modality=str(getattr(data, "modality", "image")),
        event_topic=str(getattr(data, "event_topic", "/event_camera/events")),
        event_bins=int(getattr(data, "event_bins", 10)),
        event_window_ms=float(getattr(data, "event_window_ms", 40.0)),
        event_stride_ms=float(getattr(data, "event_stride_ms", 4.0)),
        event_polarity_layout=str(getattr(data, "event_polarity_layout", "polarity_major")),
        event_temporal_interpolation=str(
            getattr(data, "event_temporal_interpolation", "none")
        ),
        sample_hz=float(getattr(data, "sample_hz", 10.0)),
    )
    metadata = extract_dataset(extract_cfg)
    print(OmegaConf.to_yaml(OmegaConf.create(metadata)))


if __name__ == "__main__":
    main()
