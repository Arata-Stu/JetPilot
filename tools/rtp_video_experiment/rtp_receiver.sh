#!/usr/bin/env bash
set -euo pipefail

CODEC="${CODEC:-${1:-h264}}"
PORT="${PORT:-5004}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
FPS="${FPS:-30}"
PAYLOAD="${PAYLOAD:-96}"
JITTER_LATENCY_MS="${JITTER_LATENCY_MS:-0}"
LOG_DIR="${LOG_DIR:-record/rtp_video}"
DISPLAY_SINK="${DISPLAY_SINK:-autovideosink}"
NO_DISPLAY="${NO_DISPLAY:-false}"
H264_DECODER="${H264_DECODER:-auto}"
H265_DECODER="${H265_DECODER:-auto}"

die() {
  echo "error: $*" >&2
  exit 1
}

is_true() {
  case "$1" in
    true|TRUE|yes|YES|1) return 0 ;;
    *) return 1 ;;
  esac
}

has_element() {
  gst-inspect-1.0 "$1" >/dev/null 2>&1
}

select_decoder() {
  local requested="$1"
  local nv="$2"
  local sw="$3"

  if [[ "$requested" != auto ]]; then
    printf '%s\n' "$requested"
  elif has_element "$sw"; then
    printf '%s\n' "$sw"
  elif has_element "$nv"; then
    printf '%s\n' "$nv"
  else
    die "decoder was not found: $sw or $nv"
  fi
}

command -v gst-launch-1.0 >/dev/null 2>&1 || die "gst-launch-1.0 was not found"

mkdir -p "$LOG_DIR"
timestamp="$(date +%Y%m%d_%H%M%S)"
log_file="${LOG_DIR}/receiver_${CODEC}_${timestamp}.log"
pipeline_file="${LOG_DIR}/receiver_${CODEC}_${timestamp}.pipeline.txt"

PIPELINE=(gst-launch-1.0 -e)

case "$CODEC" in
  raw)
    CAPS="application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)RAW,sampling=(string)RGB,depth=(string)8,width=(string)${WIDTH},height=(string)${HEIGHT},payload=(int)${PAYLOAD}"
    PIPELINE+=(
      udpsrc port="$PORT" caps="$CAPS"
      ! rtpjitterbuffer latency="$JITTER_LATENCY_MS" drop-on-latency=true
      ! queue leaky=downstream max-size-buffers=4
      ! rtpvrawdepay
    )
    ;;
  mjpeg)
    CAPS="application/x-rtp,media=video,clock-rate=90000,encoding-name=JPEG,payload=26"
    PIPELINE+=(
      udpsrc port="$PORT" caps="$CAPS"
      ! rtpjitterbuffer latency="$JITTER_LATENCY_MS" drop-on-latency=true
      ! queue leaky=downstream max-size-buffers=32
      ! rtpjpegdepay
      ! jpegdec
    )
    ;;
  h264)
    CAPS="application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=${PAYLOAD}"
    decoder="$(select_decoder "$H264_DECODER" nvv4l2decoder avdec_h264)"
    PIPELINE+=(
      udpsrc port="$PORT" caps="$CAPS"
      ! rtpjitterbuffer latency="$JITTER_LATENCY_MS" drop-on-latency=true
      ! queue leaky=downstream max-size-buffers=64
      ! rtph264depay
      ! h264parse
      ! "$decoder"
    )
    ;;
  h265)
    CAPS="application/x-rtp,media=video,clock-rate=90000,encoding-name=H265,payload=${PAYLOAD}"
    decoder="$(select_decoder "$H265_DECODER" nvv4l2decoder avdec_h265)"
    PIPELINE+=(
      udpsrc port="$PORT" caps="$CAPS"
      ! rtpjitterbuffer latency="$JITTER_LATENCY_MS" drop-on-latency=true
      ! queue leaky=downstream max-size-buffers=64
      ! rtph265depay
      ! h265parse
      ! "$decoder"
    )
    ;;
  *)
    die "unsupported CODEC: $CODEC"
    ;;
esac

PIPELINE+=(! videoconvert)
if is_true "$NO_DISPLAY"; then
  PIPELINE+=(! fakesink sync=false)
else
  PIPELINE+=(
    ! fpsdisplaysink video-sink="$DISPLAY_SINK" text-overlay=false sync=false
  )
fi

{
  printf 'PORT=%s\nCODEC=%s\nWIDTH=%s\nHEIGHT=%s\nFPS=%s\n' \
    "$PORT" "$CODEC" "$WIDTH" "$HEIGHT" "$FPS"
  printf 'Command:\n'
  printf '%q ' "${PIPELINE[@]}"
  printf '\n'
} > "$pipeline_file"

echo "Receiver pipeline:"
printf '  %q' "${PIPELINE[@]}"
echo
echo "Log: $log_file"

"${PIPELINE[@]}" 2>&1 | tee "$log_file"
