import argparse
import sys
from pathlib import Path

from event_camera_analyzer.diagnoser import FlickerDiagnoser
from event_camera_analyzer.reader import BagReader
from event_camera_analyzer.visualizer import DashboardVisualizer


def main():
    parser = argparse.ArgumentParser(
        description="ROSBag analyzer and root cause diagnosis tool for event camera flickering"
    )
    parser.add_argument(
        "--bag",
        type=str,
        required=True,
        help="Path to the ROSBag directory (.mcap / .db3)",
    )
    parser.add_argument(
        "--output-html",
        type=str,
        default="event_camera_report.html",
        help="Path to save the generated Plotly HTML dashboard (default: event_camera_report.html)",
    )
    parser.add_argument(
        "--namespace",
        type=str,
        default="/event_camera",
        help="Topic namespace prefix (default: /event_camera)",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=25.0,
        help="Expected event image frame rate (default: 25.0 Hz)",
    )
    parser.add_argument(
        "--drop-threshold",
        type=float,
        default=0.5,
        help="Active pixel ratio drop ratio threshold for flicker detection (default: 0.5)",
    )
    parser.add_argument(
        "--window-ms",
        type=float,
        default=60.0,
        help="Time window before flicker frame to evaluate packet stream (default: 60.0 ms)",
    )

    args = parser.parse_args()

    bag_path = Path(args.bag)
    if not bag_path.exists():
        print(f"[ERROR] ROSBag path does not exist: {bag_path}", file=sys.stderr)
        sys.exit(1)

    print(f"===========================================================")
    print(f" OpenEB ROSBag Analyzer v0.1.0")
    print(f"===========================================================")
    print(f"[*] Reading Bag      : {bag_path}")
    print(f"[*] Topic Namespace  : {args.namespace}")
    print(f"[*] Expected FPS     : {args.fps:.1f} Hz")

    try:
        reader = BagReader(bag_path, namespace=args.namespace)
        bag_data = reader.read_bag()
    except Exception as e:
        print(f"[ERROR] Failed to read ROSBag: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"[*] Bag Duration     : {bag_data.duration_sec:.2f} sec")
    print(f"[*] Raw Packets      : {len(bag_data.raw_packets)} (/events_raw)")
    print(f"[*] Processed Packets: {len(bag_data.processed_packets)} (/events)")
    print(f"[*] Event Images     : {len(bag_data.images)} (/event_image)")
    print(f"[*] Diagnostics      : {len(bag_data.diagnostics)} (/diagnostics)")
    print(f"-----------------------------------------------------------")

    diagnoser = FlickerDiagnoser(
        expected_fps=args.fps,
        drop_threshold=args.drop_threshold,
        window_ms=args.window_ms,
    )
    summary = diagnoser.diagnose(bag_data)

    print(f"[+] Total Images Analyzed : {summary.total_images}")
    print(f"[+] Detected Flickers     : {summary.total_flicker_events}")
    print(f"    - STREAM_STARVATION     : {summary.cause_counts.get('STREAM_STARVATION', 0)} (イベント欠落/レート低下)")
    print(f"    - TIMER_DESYNC_OR_JITTER: {summary.cause_counts.get('TIMER_DESYNC_OR_JITTER', 0)} (画像生成タイマージッター/非同期)")
    print(f"    - DECODE_ERROR          : {summary.cause_counts.get('DECODE_ERROR', 0)} (デコード例外/領域外)")
    print(f"    - UNKNOWN               : {summary.cause_counts.get('UNKNOWN', 0)}")
    print(f"-----------------------------------------------------------")
    print("【自動診断サマリーと推奨対策】")
    print(summary.recommendation)
    print(f"-----------------------------------------------------------")

    visualizer = DashboardVisualizer(bag_data, summary)
    out_html = visualizer.generate_html(args.output_html)
    print(f"[+] Interactive Report Saved to: {out_html}")
    print(f"===========================================================\n")


if __name__ == "__main__":
    main()
