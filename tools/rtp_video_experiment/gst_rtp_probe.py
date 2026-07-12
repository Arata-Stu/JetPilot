#!/usr/bin/env python3
"""Run RTP video sender/receiver pipelines and log GStreamer stage timestamps."""

from __future__ import annotations

import argparse
import csv
import os
import signal
import sys
import time
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("role", choices=("sender", "receiver"))
    parser.add_argument("--codec", choices=("raw", "mjpeg", "h264", "h265"), default="h264")
    parser.add_argument("--host", default="127.0.0.1", help="Receiver IP for sender mode")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--bitrate", type=int, default=8_000_000)
    parser.add_argument("--gop", type=int, default=30)
    parser.add_argument("--jpeg-quality", type=int, default=80)
    parser.add_argument("--mtu", type=int, default=1200)
    parser.add_argument("--payload", type=int, default=96)
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds. 0 means run until Ctrl-C")
    parser.add_argument("--log", default="", help="CSV log path")
    parser.add_argument("--pipeline-log", default="", help="Optional pipeline text path")
    parser.add_argument("--test-src", action="store_true")
    parser.add_argument("--test-pattern", default="ball")
    parser.add_argument("--display-sink", default="autovideosink")
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--encoder", default="auto", help="auto, nvv4l2h264enc, nvv4l2h265enc, x264enc, x265enc")
    parser.add_argument("--decoder", default="auto", help="auto, avdec_h264, avdec_h265, nvv4l2decoder")
    parser.add_argument("--encoder-extra", default="", help="Extra encoder properties, space separated")
    return parser.parse_args()


def ns_or_none(value: int) -> str:
    if value is None or value < 0 or value >= 18_446_744_073_709_551_615:
        return ""
    return str(int(value))


def ensure_gst():
    try:
        import gi  # type: ignore

        gi.require_version("Gst", "1.0")
        gi.require_version("GLib", "2.0")
        from gi.repository import GLib, Gst  # type: ignore
    except Exception as exc:  # pragma: no cover - depends on target machine packages
        print(f"error: PyGObject/GStreamer bindings are unavailable: {exc}", file=sys.stderr)
        sys.exit(2)

    Gst.init(None)
    return Gst, GLib


def has_element(Gst, name: str) -> bool:
    return Gst.ElementFactory.find(name) is not None


def select_encoder(Gst, codec: str, requested: str) -> str:
    if requested != "auto":
        return requested
    if codec == "h264":
        if has_element(Gst, "nvv4l2h264enc"):
            return "nvv4l2h264enc"
        if has_element(Gst, "x264enc"):
            return "x264enc"
    if codec == "h265":
        if has_element(Gst, "nvv4l2h265enc"):
            return "nvv4l2h265enc"
        if has_element(Gst, "x265enc"):
            return "x265enc"
    raise RuntimeError(f"No encoder found for {codec}")


def select_decoder(Gst, codec: str, requested: str) -> str:
    if requested != "auto":
        return requested
    software = {"h264": "avdec_h264", "h265": "avdec_h265"}.get(codec)
    if software and has_element(Gst, software):
        return software
    if codec in ("h264", "h265") and has_element(Gst, "nvv4l2decoder"):
        return "nvv4l2decoder"
    raise RuntimeError(f"No decoder found for {codec}")


def source_desc(args: argparse.Namespace) -> str:
    caps = f"video/x-raw,width={args.width},height={args.height},framerate={args.fps}/1"
    if args.test_src:
        return (
            f"videotestsrc is-live=true pattern={args.test_pattern} do-timestamp=true "
            f"! video/x-raw,format=RGB,width={args.width},height={args.height},framerate={args.fps}/1"
        )
    return (
        f"v4l2src device={args.device} io-mode=2 do-timestamp=true "
        f"! {caps} ! videoconvert ! video/x-raw,format=RGB"
    )


def sender_pipeline(Gst, args: argparse.Namespace) -> str:
    base = (
        f"{source_desc(args)} "
        "! identity name=acquire signal-handoffs=true "
        "! queue leaky=downstream max-size-buffers=1 "
        "! identity name=encode_start signal-handoffs=true "
    )

    if args.codec == "raw":
        codec_desc = (
            "! identity name=encode_done signal-handoffs=true "
            f"! rtpvrawpay pt={args.payload} mtu={args.mtu}"
        )
    elif args.codec == "mjpeg":
        codec_desc = (
            "! videoconvert ! video/x-raw,format=I420 "
            f"! jpegenc quality={args.jpeg_quality} "
            "! identity name=encode_done signal-handoffs=true "
            f"! rtpjpegpay pt=26 mtu={args.mtu}"
        )
    elif args.codec == "h264":
        encoder = select_encoder(Gst, args.codec, args.encoder)
        if encoder == "nvv4l2h264enc":
            extra = args.encoder_extra or "num-B-Frames=0 preset-level=1"
            codec_desc = (
                "! videoconvert ! video/x-raw,format=NV12 ! nvvidconv "
                "! video/x-raw(memory:NVMM),format=NV12 "
                f"! nvv4l2h264enc bitrate={args.bitrate} control-rate=1 "
                f"iframeinterval={args.gop} {extra} "
                "! h264parse config-interval=-1 "
                "! video/x-h264,alignment=au,stream-format=byte-stream "
                "! identity name=encode_done signal-handoffs=true "
                f"! rtph264pay pt={args.payload} config-interval=1 mtu={args.mtu}"
            )
        else:
            codec_desc = (
                "! videoconvert ! video/x-raw,format=I420 "
                f"! x264enc tune=zerolatency speed-preset=ultrafast bitrate={args.bitrate // 1000} "
                f"key-int-max={args.gop} bframes=0 byte-stream=true "
                "! h264parse config-interval=-1 "
                "! video/x-h264,alignment=au,stream-format=byte-stream "
                "! identity name=encode_done signal-handoffs=true "
                f"! rtph264pay pt={args.payload} config-interval=1 mtu={args.mtu}"
            )
    elif args.codec == "h265":
        encoder = select_encoder(Gst, args.codec, args.encoder)
        if encoder == "nvv4l2h265enc":
            extra = args.encoder_extra or "num-B-Frames=0 preset-level=1"
            codec_desc = (
                "! videoconvert ! video/x-raw,format=NV12 ! nvvidconv "
                "! video/x-raw(memory:NVMM),format=NV12 "
                f"! nvv4l2h265enc bitrate={args.bitrate} control-rate=1 "
                f"iframeinterval={args.gop} {extra} "
                "! h265parse config-interval=-1 "
                "! video/x-h265,alignment=au,stream-format=byte-stream "
                "! identity name=encode_done signal-handoffs=true "
                f"! rtph265pay pt={args.payload} config-interval=1 mtu={args.mtu}"
            )
        else:
            codec_desc = (
                "! videoconvert ! video/x-raw,format=I420 "
                f"! x265enc tune=zerolatency speed-preset=ultrafast bitrate={args.bitrate // 1000} "
                f"key-int-max={args.gop} option-string=bframes=0 "
                "! h265parse config-interval=-1 "
                "! video/x-h265,alignment=au,stream-format=byte-stream "
                "! identity name=encode_done signal-handoffs=true "
                f"! rtph265pay pt={args.payload} config-interval=1 mtu={args.mtu}"
            )
    else:
        raise RuntimeError(f"Unsupported codec: {args.codec}")

    return (
        f"{base} {codec_desc} "
        "! identity name=rtp_send signal-handoffs=true "
        f"! udpsink host={args.host} port={args.port} sync=false async=false"
    )


def rtp_caps(args: argparse.Namespace) -> str:
    if args.codec == "raw":
        return (
            "application/x-rtp,media=video,clock-rate=90000,encoding-name=RAW,"
            f"sampling=RGB,depth=8,width={args.width},height={args.height},payload={args.payload}"
        )
    if args.codec == "mjpeg":
        return "application/x-rtp,media=video,clock-rate=90000,encoding-name=JPEG,payload=26"
    if args.codec == "h264":
        return f"application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload={args.payload}"
    if args.codec == "h265":
        return f"application/x-rtp,media=video,clock-rate=90000,encoding-name=H265,payload={args.payload}"
    raise RuntimeError(f"Unsupported codec: {args.codec}")


def receiver_pipeline(Gst, args: argparse.Namespace) -> str:
    base = (
        f"udpsrc port={args.port} caps=\"{rtp_caps(args)}\" "
        "! identity name=rtp_recv signal-handoffs=true "
        "! queue leaky=downstream max-size-buffers=64 "
    )

    if args.codec == "raw":
        codec_desc = (
            "! rtpvrawdepay "
            "! identity name=frame_recv_done signal-handoffs=true "
            "! identity name=decode_done signal-handoffs=true "
        )
    elif args.codec == "mjpeg":
        codec_desc = (
            "! rtpjpegdepay "
            "! identity name=frame_recv_done signal-handoffs=true "
            "! jpegdec "
            "! identity name=decode_done signal-handoffs=true "
        )
    elif args.codec == "h264":
        decoder = select_decoder(Gst, args.codec, args.decoder)
        codec_desc = (
            "! rtph264depay "
            "! identity name=frame_recv_done signal-handoffs=true "
            "! h264parse "
            f"! {decoder} "
            "! identity name=decode_done signal-handoffs=true "
        )
    elif args.codec == "h265":
        decoder = select_decoder(Gst, args.codec, args.decoder)
        codec_desc = (
            "! rtph265depay "
            "! identity name=frame_recv_done signal-handoffs=true "
            "! h265parse "
            f"! {decoder} "
            "! identity name=decode_done signal-handoffs=true "
        )
    else:
        raise RuntimeError(f"Unsupported codec: {args.codec}")

    sink = "fakesink sync=false" if args.no_display else (
        f"fpsdisplaysink video-sink={args.display_sink} text-overlay=false sync=false"
    )
    return (
        f"{base} {codec_desc} "
        "! videoconvert "
        "! identity name=render_submit signal-handoffs=true "
        f"! {sink}"
    )


class CsvLogger:
    def __init__(self, path: str):
        if not path:
            stamp = time.strftime("%Y%m%d_%H%M%S")
            path = f"record/rtp_video/rtp_probe_{stamp}.csv"
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(
            self.file,
            fieldnames=[
                "wall_ns",
                "mono_ns",
                "role",
                "codec",
                "stage",
                "seq",
                "pts_ns",
                "dts_ns",
                "duration_ns",
                "offset",
                "offset_end",
                "size_bytes",
            ],
        )
        self.writer.writeheader()
        self.counts: dict[str, int] = {}

    def log_buffer(self, role: str, codec: str, stage: str, buffer) -> None:
        seq = self.counts.get(stage, 0)
        self.counts[stage] = seq + 1
        self.writer.writerow(
            {
                "wall_ns": time.time_ns(),
                "mono_ns": time.monotonic_ns(),
                "role": role,
                "codec": codec,
                "stage": stage,
                "seq": seq,
                "pts_ns": ns_or_none(buffer.pts),
                "dts_ns": ns_or_none(buffer.dts),
                "duration_ns": ns_or_none(buffer.duration),
                "offset": "" if buffer.offset < 0 else str(buffer.offset),
                "offset_end": "" if buffer.offset_end < 0 else str(buffer.offset_end),
                "size_bytes": buffer.get_size(),
            }
        )
        self.file.flush()

    def close(self) -> None:
        self.file.close()


def run() -> int:
    args = parse_args()
    Gst, GLib = ensure_gst()

    try:
        pipeline_desc = sender_pipeline(Gst, args) if args.role == "sender" else receiver_pipeline(Gst, args)
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    logger = CsvLogger(args.log)
    pipeline_log = Path(args.pipeline_log) if args.pipeline_log else logger.path.with_suffix(".pipeline.txt")
    pipeline_log.write_text(pipeline_desc + "\n", encoding="utf-8")

    print("Pipeline:")
    print(pipeline_desc)
    print(f"CSV log: {logger.path}")

    pipeline = Gst.parse_launch(pipeline_desc)

    stages = (
        "acquire",
        "encode_start",
        "encode_done",
        "rtp_send",
        "rtp_recv",
        "frame_recv_done",
        "decode_done",
        "render_submit",
    )

    def on_handoff(_identity, buffer, stage: str):
        logger.log_buffer(args.role, args.codec, stage, buffer)

    for stage in stages:
        element = pipeline.get_by_name(stage)
        if element is not None:
            element.connect("handoff", on_handoff, stage)

    loop = GLib.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()

    def on_message(_bus, message):
        if message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"error: {err}", file=sys.stderr)
            if debug:
                print(debug, file=sys.stderr)
            loop.quit()
        elif message.type == Gst.MessageType.EOS:
            loop.quit()

    bus.connect("message", on_message)

    def stop(_signum=None, _frame=None):
        pipeline.send_event(Gst.Event.new_eos())
        return False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    if args.duration > 0:
        GLib.timeout_add(int(args.duration * 1000), stop)

    pipeline.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    finally:
        pipeline.set_state(Gst.State.NULL)
        logger.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
