from pathlib import Path
from typing import List, Union

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from event_camera_analyzer.diagnoser import DiagnosisSummary, FlickerEvent
from event_camera_analyzer.reader import BagData


class DashboardVisualizer:
    """
    Generates an interactive Plotly HTML dashboard displaying time-series streams of
    events_raw, event_image, diagnostics, along with flicker detection points and root cause attribution.
    """

    def __init__(self, bag_data: BagData, summary: DiagnosisSummary):
        self.bag_data = bag_data
        self.summary = summary
        self.start_t = bag_data.start_time_ns

    def _to_sec(self, ns: int) -> float:
        return (ns - self.start_t) / 1e9

    def generate_html(self, output_path: Union[str, Path]) -> str:
        output_path = Path(output_path)
        fig = make_subplots(
            rows=4,
            cols=1,
            shared_xaxes=True,
            vertical_spacing=0.06,
            subplot_titles=(
                "1. イベントストリームレート (MEV/s & パケットサイズ)",
                "2. イベント画像 品質指標 (非背景ピクセル率 % & チラつき検知マーカー)",
                "3. パケット到着間隔 ΔT (ms) [通信・タイマージッター監視]",
                "4. 内部診断統計 (QoS破棄・負荷率・デコードエラー)",
            ),
        )

        # -------------------------------------------------------------
        # Row 1: Event Packets Rate & Volume
        # -------------------------------------------------------------
        packets = (
            self.bag_data.raw_packets
            if self.bag_data.raw_packets
            else self.bag_data.processed_packets
        )
        if packets:
            t_pkt = [self._to_sec(p.arrival_time_ns) for p in packets]
            # Calculate instantaneous MEV/s over moving pairs
            mev_s = []
            for i in range(len(packets)):
                if i == 0:
                    mev_s.append(0.0)
                else:
                    dt = (packets[i].arrival_time_ns - packets[i - 1].arrival_time_ns) / 1e9
                    ev = packets[i].estimated_events
                    mev_s.append((ev / 1e6) / dt if dt > 1e-6 else 0.0)

            fig.add_trace(
                go.Scatter(
                    x=t_pkt,
                    y=mev_s,
                    mode="lines",
                    name="イベントレート (MEV/s)",
                    line=dict(color="#00D2FF", width=1.5),
                    hovertemplate="時刻: %{x:.3f}s<br>レート: %{y:.2f} MEV/s<extra></extra>",
                ),
                row=1,
                col=1,
            )

        # -------------------------------------------------------------
        # Row 2: Event Image Active Ratio & Flicker Markers
        # -------------------------------------------------------------
        if self.bag_data.images:
            t_img = [self._to_sec(img.arrival_time_ns) for img in self.bag_data.images]
            active_pct = [img.active_pixel_ratio * 100.0 for img in self.bag_data.images]
            brightness = [img.mean_brightness for img in self.bag_data.images]

            fig.add_trace(
                go.Scatter(
                    x=t_img,
                    y=active_pct,
                    mode="lines+markers",
                    name="非背景アクティブピクセル率 (%)",
                    line=dict(color="#00E676", width=2),
                    marker=dict(size=4, color="#00E676"),
                    hovertemplate="時刻: %{x:.3f}s<br>アクティブ率: %{y:.2f}%<extra></extra>",
                ),
                row=2,
                col=1,
            )

            fig.add_trace(
                go.Scatter(
                    x=t_img,
                    y=brightness,
                    mode="lines",
                    name="平均ピクセル輝度",
                    line=dict(color="#7C4DFF", width=1, dash="dot"),
                    hovertemplate="時刻: %{x:.3f}s<br>平均輝度: %{y:.1f}<extra></extra>",
                ),
                row=2,
                col=1,
            )

            # Flicker Markers by cause
            cause_colors = {
                "STREAM_STARVATION": "#FF1744",  # Bright Red
                "TIMER_DESYNC_OR_JITTER": "#FF9100",  # Orange
                "DECODE_ERROR": "#FFEA00",  # Yellow
                "UNKNOWN": "#B0BEC5",  # Gray
            }

            for cause, color in cause_colors.items():
                evs = [e for e in self.summary.flicker_events if e.cause == cause]
                if not evs:
                    continue
                t_ev = [self._to_sec(e.arrival_time_ns) for e in evs]
                y_ev = [e.active_pixel_ratio * 100.0 for e in evs]
                hovers = [
                    f"<b>判定原因: {e.cause}</b><br>"
                    f"時刻: {self._to_sec(e.arrival_time_ns):.3f}s<br>"
                    f"アクティブピクセル率: {e.active_pixel_ratio*100:.2f}% (通常時: {e.baseline_pixel_ratio*100:.2f}%)<br>"
                    f"直前パケット数: {e.window_num_packets} (最大間隔: {e.window_max_inter_arrival_ms:.1f}ms)<br>"
                    f"直前イベントレート: {e.window_mev_s:.2f} MEV/s<br>"
                    f"<i>{e.explanation}</i>"
                    for e in evs
                ]

                fig.add_trace(
                    go.Scatter(
                        x=t_ev,
                        y=y_ev,
                        mode="markers",
                        name=f"チラつき判定 [{cause}]",
                        marker=dict(
                            size=12,
                            color=color,
                            symbol="diamond-open",
                            line=dict(width=2.5, color=color),
                        ),
                        text=hovers,
                        hovertemplate="%{text}<extra></extra>",
                    ),
                    row=2,
                    col=1,
                )

        # -------------------------------------------------------------
        # Row 3: Inter-Arrival Time ΔT (ms)
        # -------------------------------------------------------------
        if packets and len(packets) > 1:
            t_pkt_dt = [self._to_sec(packets[i].arrival_time_ns) for i in range(1, len(packets))]
            dt_pkt_ms = [
                (packets[i].arrival_time_ns - packets[i - 1].arrival_time_ns) / 1e6
                for i in range(1, len(packets))
            ]
            fig.add_trace(
                go.Scatter(
                    x=t_pkt_dt,
                    y=dt_pkt_ms,
                    mode="lines",
                    name="パケット間隔 ΔT_events (ms)",
                    line=dict(color="#29B6F6", width=1),
                    hovertemplate="時刻: %{x:.3f}s<br>ΔT_events: %{y:.2f}ms<extra></extra>",
                ),
                row=3,
                col=1,
            )

        if self.bag_data.images and len(self.bag_data.images) > 1:
            t_img_dt = [
                self._to_sec(self.bag_data.images[i].arrival_time_ns)
                for i in range(1, len(self.bag_data.images))
            ]
            dt_img_ms = [
                (self.bag_data.images[i].arrival_time_ns - self.bag_data.images[i - 1].arrival_time_ns) / 1e6
                for i in range(1, len(self.bag_data.images))
            ]
            fig.add_trace(
                go.Scatter(
                    x=t_img_dt,
                    y=dt_img_ms,
                    mode="lines+markers",
                    name="画像生成間隔 ΔT_image (ms)",
                    line=dict(color="#FF4081", width=1.5),
                    marker=dict(size=4),
                    hovertemplate="時刻: %{x:.3f}s<br>ΔT_image: %{y:.2f}ms<extra></extra>",
                ),
                row=3,
                col=1,
            )

        # -------------------------------------------------------------
        # Row 4: Preprocessor Diagnostics (Busy %, Drops, Errors)
        # -------------------------------------------------------------
        if self.bag_data.diagnostics:
            t_diag = [self._to_sec(d.arrival_time_ns) for d in self.bag_data.diagnostics]
            busy_pct = [d.values.get("callback_busy_pct", 0.0) for d in self.bag_data.diagnostics]
            drops = [
                d.values.get("dropped_empty", 0) + d.values.get("dropped_encoding", 0)
                for d in self.bag_data.diagnostics
            ]
            decode_errs = [d.values.get("decode_errors", 0) for d in self.bag_data.diagnostics]

            fig.add_trace(
                go.Scatter(
                    x=t_diag,
                    y=busy_pct,
                    mode="lines",
                    name="コールバック負荷率 callback_busy_pct (%)",
                    line=dict(color="#FFAB00", width=1.5),
                    hovertemplate="時刻: %{x:.3f}s<br>負荷率: %{y:.1f}%<extra></extra>",
                ),
                row=4,
                col=1,
            )

            fig.add_trace(
                go.Scatter(
                    x=t_diag,
                    y=drops,
                    mode="lines+markers",
                    name="QoS/エンコーディング破棄パケット数",
                    line=dict(color="#FF5252", width=1.5),
                    hovertemplate="時刻: %{x:.3f}s<br>破棄数: %{y}<extra></extra>",
                ),
                row=4,
                col=1,
            )

            fig.add_trace(
                go.Scatter(
                    x=t_diag,
                    y=decode_errs,
                    mode="lines+markers",
                    name="デコードエラー数",
                    line=dict(color="#E040FB", width=1.5),
                    hovertemplate="時刻: %{x:.3f}s<br>デコードエラー: %{y}<extra></extra>",
                ),
                row=4,
                col=1,
            )

        # Layout styling
        fig.update_layout(
            title=dict(
                text=(
                    f"<b>イベントカメラ ROSBag 診断ダッシュボード</b><br>"
                    f"<span style='font-size:13px; color:#A0A0A0;'>"
                    f"総フレーム数: {self.summary.total_images} | "
                    f"チラつき検知: {self.summary.total_flicker_events} 件 "
                    f"(Stream Starvation: {self.summary.cause_counts.get('STREAM_STARVATION',0)}, "
                    f"Timer Desync/Jitter: {self.summary.cause_counts.get('TIMER_DESYNC_OR_JITTER',0)}, "
                    f"Decode Error: {self.summary.cause_counts.get('DECODE_ERROR',0)}) | "
                    f"平均レート: {self.summary.avg_mev_s:.2f} MEV/s, {self.summary.avg_fps:.1f} FPS"
                    f"</span>"
                ),
                x=0.02,
                xanchor="left",
            ),
            template="plotly_dark",
            height=950,
            hovermode="x unified",
            legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
            margin=dict(t=110, b=50, l=60, r=40),
        )

        # Update y-axis labels
        fig.update_yaxes(title_text="MEV/s", row=1, col=1)
        fig.update_yaxes(title_text="アクティブ率 %", row=2, col=1)
        fig.update_yaxes(title_text="ΔT (ms)", row=3, col=1)
        fig.update_yaxes(title_text="値 / 負荷 %", row=4, col=1)
        fig.update_xaxes(title_text="Bag 開始からの経過時間 (秒)", row=4, col=1)

        # Write HTML with recommendation header inside
        recommendation_html = (
            f"<div style='background-color:#1E1E24; padding:16px; margin-bottom:12px; border-left: 5px solid #00D2FF; font-family:sans-serif; color:#FFFFFF; border-radius:4px;'>"
            f"<h3 style='margin-top:0; color:#00D2FF;'>自動診断サマリーと推奨対策</h3>"
            f"<pre style='font-family:monospace; white-space:pre-wrap; margin-bottom:0; color:#E0E0E0;'>{self.summary.recommendation}</pre>"
            f"</div>"
        )

        html_content = fig.to_html(full_html=True, include_plotlyjs="cdn")
        # Insert summary box right after opening body tag
        body_idx = html_content.find("<body>")
        if body_idx != -1:
            insert_pos = body_idx + len("<body>")
            html_content = (
                html_content[:insert_pos]
                + f"<div style='margin:20px;'>{recommendation_html}</div>"
                + html_content[insert_pos:]
            )

        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(html_content, encoding="utf-8")
        return str(output_path.absolute())
