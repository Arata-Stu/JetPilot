from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
from event_camera_analyzer.reader import (
    BagData,
    DiagnosticRecord,
    EventImageRecord,
    EventPacketRecord,
)


@dataclass
class FlickerEvent:
    image_index: int
    arrival_time_ns: int
    stamp_ns: int
    frame_id: str
    active_pixel_ratio: float
    baseline_pixel_ratio: float
    drop_ratio: float  # active_pixel_ratio / baseline_pixel_ratio
    mean_brightness: float
    delta_t_from_prev_ms: float
    cause: str = "UNKNOWN"
    confidence: str = "LOW"
    explanation: str = ""
    window_num_packets: int = 0
    window_total_events: int = 0
    window_max_inter_arrival_ms: float = 0.0
    window_mev_s: float = 0.0


@dataclass
class DiagnosisSummary:
    total_images: int
    total_flicker_events: int
    cause_counts: dict = field(default_factory=dict)
    flicker_events: List[FlickerEvent] = field(default_factory=list)
    avg_fps: float = 0.0
    avg_mev_s: float = 0.0
    recommendation: str = ""


class FlickerDiagnoser:
    """
    Detects event image flicker events and attributes root cause to either:
    - STREAM_STARVATION (Events/raw packet drops, inter-arrival spikes, low event rate)
    - TIMER_DESYNC_OR_JITTER (Wall-clock timer jitter / async accumulation window desync)
    - DECODE_ERROR (Codec / out-of-bounds event errors)
    """

    def __init__(
        self,
        expected_fps: float = 25.0,
        drop_threshold: float = 0.5,
        window_ms: float = 60.0,
    ):
        self.expected_fps = expected_fps
        self.drop_threshold = drop_threshold
        self.window_ns = int(window_ms * 1_000_000)
        self.expected_interval_ms = 1000.0 / expected_fps if expected_fps > 0 else 40.0

    def diagnose(self, bag_data: BagData) -> DiagnosisSummary:
        summary = DiagnosisSummary(
            total_images=len(bag_data.images),
            total_flicker_events=0,
            cause_counts={
                "STREAM_STARVATION": 0,
                "TIMER_DESYNC_OR_JITTER": 0,
                "DECODE_ERROR": 0,
                "UNKNOWN": 0,
            },
        )

        if not bag_data.images:
            summary.recommendation = "BAGファイル内にイベント画像 (/event_camera/event_image) が見つかりませんでした。"
            return summary

        # Calculate overall baseline MEV/s and image FPS
        packets = bag_data.raw_packets if bag_data.raw_packets else bag_data.processed_packets
        if packets and bag_data.duration_sec > 0:
            total_ev = sum(p.estimated_events for p in packets)
            summary.avg_mev_s = (total_ev / 1e6) / bag_data.duration_sec

        if len(bag_data.images) > 1:
            duration_img_s = (bag_data.images[-1].arrival_time_ns - bag_data.images[0].arrival_time_ns) / 1e9
            if duration_img_s > 0:
                summary.avg_fps = (len(bag_data.images) - 1) / duration_img_s

        # Rolling baseline calculation for active_pixel_ratio
        ratios = [img.active_pixel_ratio for img in bag_data.images]
        window_size = 5

        for i, img in enumerate(bag_data.images):
            # Calculate local median baseline from surrounding frames (excluding current)
            start_idx = max(0, i - window_size)
            end_idx = min(len(bag_data.images), i + window_size + 1)
            neighbors = ratios[start_idx:i] + ratios[i + 1 : end_idx]
            baseline = float(np.median(neighbors)) if neighbors else 0.01

            delta_t_prev_ms = (
                (img.arrival_time_ns - bag_data.images[i - 1].arrival_time_ns) / 1e6
                if i > 0
                else self.expected_interval_ms
            )

            # Check if this frame is a flicker candidate
            # Criteria: drop below threshold compared to baseline, or absolute near-zero when baseline > 0.01
            is_flicker = False
            drop_ratio = img.active_pixel_ratio / baseline if baseline > 1e-6 else 1.0

            if baseline > 0.008 and drop_ratio < self.drop_threshold:
                is_flicker = True
            elif baseline > 0.015 and img.active_pixel_ratio < 0.003:
                is_flicker = True
            elif delta_t_prev_ms > self.expected_interval_ms * 1.8 and i > 0:
                # Large gap between frames
                is_flicker = True

            if is_flicker:
                event = FlickerEvent(
                    image_index=i,
                    arrival_time_ns=img.arrival_time_ns,
                    stamp_ns=img.stamp_ns,
                    frame_id=img.frame_id,
                    active_pixel_ratio=img.active_pixel_ratio,
                    baseline_pixel_ratio=baseline,
                    drop_ratio=drop_ratio,
                    mean_brightness=img.mean_brightness,
                    delta_t_from_prev_ms=delta_t_prev_ms,
                )
                self._attribute_root_cause(event, bag_data, packets)
                summary.flicker_events.append(event)
                summary.cause_counts[event.cause] = summary.cause_counts.get(event.cause, 0) + 1

        summary.total_flicker_events = len(summary.flicker_events)
        summary.recommendation = self._generate_recommendation(summary)
        return summary

    def _attribute_root_cause(
        self,
        event: FlickerEvent,
        bag_data: BagData,
        packets: List[EventPacketRecord],
    ) -> None:
        win_start = event.arrival_time_ns - self.window_ns
        win_end = event.arrival_time_ns

        # Filter packets inside the time window
        window_packets = [p for p in packets if win_start <= p.arrival_time_ns <= win_end]
        event.window_num_packets = len(window_packets)
        event.window_total_events = sum(p.estimated_events for p in window_packets)

        if len(window_packets) >= 2:
            intervals = [
                (window_packets[k].arrival_time_ns - window_packets[k - 1].arrival_time_ns) / 1e6
                for k in range(1, len(window_packets))
            ]
            event.window_max_inter_arrival_ms = max(intervals) if intervals else 0.0
        elif len(window_packets) == 1:
            event.window_max_inter_arrival_ms = (win_end - window_packets[0].arrival_time_ns) / 1e6
        else:
            event.window_max_inter_arrival_ms = self.window_ns / 1e6

        event.window_mev_s = (
            (event.window_total_events / 1e6) / (self.window_ns / 1e9)
            if self.window_ns > 0
            else 0.0
        )

        # Check diagnostics around this timestamp (+/- 500ms)
        diag_start = event.arrival_time_ns - 500_000_000
        diag_end = event.arrival_time_ns + 500_000_000
        near_diags = [d for d in bag_data.diagnostics if diag_start <= d.arrival_time_ns <= diag_end]

        has_decode_errors = False
        decode_err_msg = ""
        for d in near_diags:
            if d.values.get("decode_errors", 0) > 0 or d.values.get("out_of_bounds_events", 0) > 0:
                has_decode_errors = True
                decode_err_msg = f"decode_errors={d.values.get('decode_errors',0)}, out_of_bounds={d.values.get('out_of_bounds_events',0)}"
                break

        # Decision Logic
        if has_decode_errors:
            event.cause = "DECODE_ERROR"
            event.confidence = "HIGH"
            event.explanation = (
                f"プリプロセッサ側の診断トピックでデコードエラーまたは領域外イベントが検出されました ({decode_err_msg})。画像描画バッファが破棄/リセットされたことが原因です。"
            )
        elif event.window_num_packets == 0 or event.window_max_inter_arrival_ms >= 32.0 or event.window_mev_s < 0.05:
            event.cause = "STREAM_STARVATION"
            event.confidence = "HIGH" if event.window_num_packets == 0 or event.window_max_inter_arrival_ms >= 40.0 else "MEDIUM"
            if event.window_num_packets == 0:
                event.explanation = (
                    f"チラつき直前の {self.window_ns/1e6:.1f}ms ウィンドウ間でイベントパケット (/events または /events_raw) が1つも到着していません。カメラ・通信・ドライバ側でのストリーム途絶/遅延が原因です。"
                )
            else:
                event.explanation = (
                    f"チラつき直前のウィンドウ間で最大パケット間隔 {event.window_max_inter_arrival_ms:.1f}ms のスパイク（遅延/欠落）またはイベントレート激減 ({event.window_mev_s:.2f} MEV/s) が発生しました。カメラ側/USB帯域起因の可能性が高いです。"
                )
        elif event.window_num_packets >= 2 and event.window_max_inter_arrival_ms < 28.0 and event.window_mev_s >= 0.1:
            event.cause = "TIMER_DESYNC_OR_JITTER"
            event.confidence = "HIGH"
            event.explanation = (
                f"直前 {self.window_ns/1e6:.1f}ms 間で {event.window_num_packets} 個のパケット (合計 {event.window_total_events} イベント, {event.window_mev_s:.2f} MEV/s) が最大間隔 {event.window_max_inter_arrival_ms:.1f}ms で定常的に到着しています。カメラストリーム側 (/events_raw, /events) に問題はありません。\n"
                f"原因はプリプロセッサ側 (on_event_image_timer) の実時間タイマージッターまたはパケット到着バーストとの非同期干渉により、画像への累積期間が短縮されたことです。"
            )
        else:
            event.cause = "UNKNOWN"
            event.confidence = "LOW"
            event.explanation = (
                f"直前ウィンドウ内に {event.window_num_packets} パケット, {event.window_mev_s:.2f} MEV/s が記録されています。急落率 {event.drop_ratio:.2f}。極端な遅延は確認されませんでした。"
            )

    def _generate_recommendation(self, summary: DiagnosisSummary) -> str:
        if summary.total_flicker_events == 0:
            return "チラつき（極端な輝度急落・アクティブピクセルのドロップ）は検出されませんでした。"

        stream_cnt = summary.cause_counts.get("STREAM_STARVATION", 0)
        timer_cnt = summary.cause_counts.get("TIMER_DESYNC_OR_JITTER", 0)
        decode_cnt = summary.cause_counts.get("DECODE_ERROR", 0)

        if timer_cnt > stream_cnt and timer_cnt > decode_cnt:
            return (
                f"【診断結果: 画像生成・タイマー起因 (TIMER_DESYNC_OR_JITTER) が優勢 ({timer_cnt}件 / 全{summary.total_flicker_events}件)】\n"
                "  イベントストリーム (/events_raw, /events) は定常的に到着していますが、プリプロセッサの Wall-clock タイマー (on_event_image_timer) と非同期パケット受信の干渉で画像がチラついています。\n"
                "  推奨対策: openeb_ros2 の composed.launch.py にて QoS キューサイズの調整、またはタイマー駆動ではなくセンサータイムスタンプ同期での画像累積（フレーム境界パケット同期方式）へのリファクタリングを推奨します。"
            )
        elif stream_cnt >= timer_cnt and stream_cnt > 0:
            return (
                f"【診断結果: カメラストリーム起因 (STREAM_STARVATION) が優勢 ({stream_cnt}件 / 全{summary.total_flicker_events}件)】\n"
                "  /events_raw または /events パケット自体の到着間隔にスパイク（欠落や帯域不足）が生じており、それが画像チラつきに直結しています。\n"
                "  推奨対策: USB 3.0 コントローラの帯域・バッファ負荷、ドライバノード (`openeb_driver_node`) の QoS / CPU アフィニティ、またはセンサバイアスパラメータ (`bias_diff_on/off`) の確認を推奨します。"
            )
        elif decode_cnt > 0:
            return (
                f"【診断結果: デコードエラー起因 (DECODE_ERROR) ({decode_cnt}件 / 全{summary.total_flicker_events}件)】\n"
                "  デコード時の例外や領域外イベントによりバッファ破棄が発生しています。"
            )
        else:
            return f"全 {summary.total_flicker_events} 件のチラつきを検出しました。詳細なタイムラインをご確認ください。"
