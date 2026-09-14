from __future__ import annotations

import collections
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class EventTensorConfig:
    bins: int = 10
    window_ms: float = 40.0
    stride_ms: float = 4.0
    width: int = 212
    height: int = 120
    polarity_layout: str = "polarity_major"
    temporal_interpolation: str = "none"

    @property
    def channels(self) -> int:
        return 2 * self.bins


class EventTensorAccumulator:
    """CPU reference for the runtime C++/CUDA rolling event representation."""

    def __init__(self, config: EventTensorConfig) -> None:
        if config.bins < 1 or config.bins > 64:
            raise ValueError("event bins must be between 1 and 64")
        if config.window_ms <= 0.0 or config.stride_ms <= 0.0:
            raise ValueError("event window and stride must be positive")
        if config.width < 1 or config.height < 1:
            raise ValueError("event output dimensions must be positive")
        if config.polarity_layout not in {"polarity_major", "time_major"}:
            raise ValueError("polarity layout must be polarity_major or time_major")
        if config.temporal_interpolation not in {"none", "linear"}:
            raise ValueError("temporal interpolation must be none or linear")
        self.config = config
        self.window_ns = int(round(config.window_ms * 1_000_000.0))
        self.stride_ns = int(round(config.stride_ms * 1_000_000.0))
        self.source_width = 0
        self.source_height = 0
        self.first_event_ns: int | None = None
        self.latest_event_ns: int | None = None
        self.chunks: collections.deque[
            tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]
        ] = collections.deque()
        self.timestamp_resets = 0
        self.previous_window_end_ns: int | None = None
        self.previous_tensor: np.ndarray | None = None

    def add(
        self,
        sensor_us: np.ndarray,
        x: np.ndarray,
        y: np.ndarray,
        polarity: np.ndarray,
        *,
        source_width: int,
        source_height: int,
    ) -> None:
        if len(sensor_us) == 0:
            return
        event_ns = np.asarray(sensor_us, dtype=np.int64) * 1000
        owned_x = np.asarray(x, dtype=np.int32).copy()
        owned_y = np.asarray(y, dtype=np.int32).copy()
        owned_polarity = np.asarray(polarity, dtype=np.bool_).copy()
        if self.latest_event_ns is not None and int(event_ns[0]) < self.latest_event_ns:
            self.chunks.clear()
            self.first_event_ns = None
            self.previous_window_end_ns = None
            self.previous_tensor = None
            self.timestamp_resets += 1
        self.source_width = int(source_width)
        self.source_height = int(source_height)
        self.chunks.append((event_ns, owned_x, owned_y, owned_polarity))
        if self.first_event_ns is None:
            self.first_event_ns = int(event_ns[0])
        self.latest_event_ns = int(event_ns[-1])
        oldest = self.latest_event_ns - self.window_ns - self.stride_ns
        while self.chunks and int(self.chunks[0][0][-1]) < oldest:
            self.chunks.popleft()

    def add_packet(self, decoder: Any, message: Any) -> int:
        decoder.decode_bytes(
            str(message.encoding), int(message.width), int(message.height),
            int(message.time_base), bytes(message.events),
        )
        decoded = decoder.get_cd_events()
        if decoded is None or len(decoded) == 0:
            return 0
        self.add(
            decoded["t"], decoded["x"], decoded["y"], decoded["p"],
            source_width=int(message.width), source_height=int(message.height),
        )
        return len(decoded)

    def snapshot(self) -> tuple[np.ndarray, dict[str, int | float]] | None:
        if self.latest_event_ns is None or self.first_event_ns is None:
            return None
        window_end_ns = self.first_event_ns + (
            (self.latest_event_ns - self.first_event_ns) // self.stride_ns
        ) * self.stride_ns
        if window_end_ns - self.first_event_ns < self.window_ns:
            return None
        start_ns = window_end_ns - self.window_ns
        cfg = self.config
        if self.previous_window_end_ns == window_end_ns and self.previous_tensor is not None:
            return self.previous_tensor, {
                "events": int(round(float(self.previous_tensor.sum(dtype=np.float64)))),
                "window_start_sensor_ns": start_ns,
                "window_end_sensor_ns": window_end_ns,
                "timestamp_resets": self.timestamp_resets,
            }
        bin_width_ns = self.window_ns // cfg.bins if self.window_ns % cfg.bins == 0 else 0
        incremental = (
            cfg.temporal_interpolation == "none"
            and bin_width_ns > 0
            and self.stride_ns == bin_width_ns
            and self.previous_tensor is not None
            and self.previous_window_end_ns is not None
            and window_end_ns - self.previous_window_end_ns == self.stride_ns
        )
        if incremental:
            tensor = np.empty_like(self.previous_tensor)
            if cfg.polarity_layout == "polarity_major":
                tensor[: cfg.bins - 1] = self.previous_tensor[1 : cfg.bins]
                tensor[cfg.bins - 1] = 0.0
                tensor[cfg.bins : 2 * cfg.bins - 1] = self.previous_tensor[
                    cfg.bins + 1 : 2 * cfg.bins
                ]
                tensor[2 * cfg.bins - 1] = 0.0
            else:
                tensor[:-2] = self.previous_tensor[2:]
                tensor[-2:] = 0.0
            accumulation_start_ns = self.previous_window_end_ns
        else:
            tensor = np.zeros((cfg.channels, cfg.height, cfg.width), dtype=np.float32)
            accumulation_start_ns = start_ns
        event_count = 0
        for times, source_x, source_y, polarity in self.chunks:
            selected = (times >= accumulation_start_ns) & (times < window_end_ns)
            if not bool(selected.any()):
                continue
            times_selected = times[selected]
            sx = source_x[selected]
            sy = source_y[selected]
            selected_polarity = polarity[selected]
            valid = (
                (sx >= 0) & (sx < self.source_width)
                & (sy >= 0) & (sy < self.source_height)
            )
            if not bool(valid.any()):
                continue
            times_selected = times_selected[valid]
            sx = sx[valid]
            sy = sy[valid]
            selected_polarity = selected_polarity[valid]
            target_x = np.minimum(
                cfg.width - 1, sx.astype(np.int64) * cfg.width // max(self.source_width, 1)
            )
            target_y = np.minimum(
                cfg.height - 1, sy.astype(np.int64) * cfg.height // max(self.source_height, 1)
            )
            polarity_index = np.where(selected_polarity, 0, 1).astype(np.int64)
            if incremental:
                temporal_bin = np.full(len(times_selected), cfg.bins - 1, dtype=np.int64)
                channel = self._channels(temporal_bin, polarity_index)
                np.add.at(tensor, (channel, target_y, target_x), 1.0)
            elif cfg.temporal_interpolation == "linear" and cfg.bins > 1:
                position = (
                    (times_selected - start_ns).astype(np.float64)
                    * (cfg.bins - 1) / self.window_ns
                )
                lower = np.floor(position).astype(np.int64)
                upper = np.minimum(lower + 1, cfg.bins - 1)
                upper_weight = (position - lower).astype(np.float32)
                lower_channel = self._channels(lower, polarity_index)
                upper_channel = self._channels(upper, polarity_index)
                np.add.at(tensor, (lower_channel, target_y, target_x), 1.0 - upper_weight)
                np.add.at(tensor, (upper_channel, target_y, target_x), upper_weight)
            else:
                temporal_bin = np.minimum(
                    cfg.bins - 1,
                    (times_selected - start_ns) * cfg.bins // self.window_ns,
                ).astype(np.int64)
                channel = self._channels(temporal_bin, polarity_index)
                np.add.at(tensor, (channel, target_y, target_x), 1.0)
            event_count += int(valid.sum())
        # Separate-polarity representations are non-negative, so their sum is
        # the exact number of events in the complete rolling window even when
        # only the newest bin was accumulated above.
        event_count = int(round(float(tensor.sum(dtype=np.float64))))
        self.previous_window_end_ns = window_end_ns
        self.previous_tensor = tensor
        return tensor, {
            "events": event_count,
            "window_start_sensor_ns": start_ns,
            "window_end_sensor_ns": window_end_ns,
            "timestamp_resets": self.timestamp_resets,
        }

    def _channels(self, temporal_bin: np.ndarray, polarity_index: np.ndarray) -> np.ndarray:
        if self.config.polarity_layout == "polarity_major":
            return polarity_index * self.config.bins + temporal_bin
        return temporal_bin * 2 + polarity_index
