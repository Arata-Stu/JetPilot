#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

CODEC="${CODEC:-${1:-h264}}"
HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-5000}"
DEVICE="${DEVICE:-/dev/video0}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
FPS="${FPS:-30}"
BITRATE="${BITRATE:-8000000}"
GOP="${GOP:-30}"
JPEG_QUALITY="${JPEG_QUALITY:-80}"
MTU="${MTU:-1200}"
PAYLOAD="${PAYLOAD:-96}"
USE_TESTSRC="${USE_TESTSRC:-false}"
TEST_PATTERN="${TEST_PATTERN:-ball}"
LOG_DIR="${LOG_DIR:-record/rtp_video}"
H264_ENCODER="${H264_ENCODER:-auto}"
H265_ENCODER="${H265_ENCODER:-auto}"
H264_ENCODER_EXTRA="${H264_ENCODER_EXTRA:-num-B-Frames=0 preset-level=1}"
H265_ENCODER_EXTRA="${H265_ENCODER_EXTRA:-num-B-Frames=0 preset-level=1}"

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

select_h264_encoder() {
  if [[ "$H264_ENCODER" != auto ]]; then
    printf '%s\n' "$H264_ENCODER"
  elif has_element nvv4l2h264enc; then
    printf '%s\n' nvv4l2h264enc
  elif has_element x264enc; then
    printf '%s\n' x264enc
  else
    die "No H.264 encoder found. Install nvv4l2h264enc or x264enc."
  fi
}

select_h265_encoder() {
  if [[ "$H265_ENCODER" != auto ]]; then
    printf '%s\n' "$H265_ENCODER"
  elif has_element nvv4l2h265enc; then
    printf '%s\n' nvv4l2h265enc
  elif has_element x265enc; then
    printf '%s\n' x265enc
  else
    die "No H.265 encoder found. Install nvv4l2h265enc or x265enc."
  fi
}

append_source() {
  if is_true "$USE_TESTSRC"; then
    PIPELINE+=(
      videotestsrc is-live=true pattern="$TEST_PATTERN" do-timestamp=true
      ! "video/x-raw,format=RGB,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1"
    )
  else
    [[ -e "$DEVICE" ]] || die "video device does not exist: $DEVICE"
    PIPELINE+=(
      v4l2src device="$DEVICE" io-mode=2 do-timestamp=true
      ! "video/x-raw,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1"
      ! videoconvert
      ! "video/x-raw,format=RGB"
    )
  fi
  PIPELINE+=(! queue leaky=downstream max-size-buffers=1)
}

append_udp_sink() {
  PIPELINE+=(
    ! udpsink host="$HOST" port="$PORT" sync=false async=false
  )
}

command -v gst-launch-1.0 >/dev/null 2>&1 || die "gst-launch-1.0 was not found"

mkdir -p "$LOG_DIR"
timestamp="$(date +%Y%m%d_%H%M%S)"
log_file="${LOG_DIR}/sender_${CODEC}_${timestamp}.log"
pipeline_file="${LOG_DIR}/sender_${CODEC}_${timestamp}.pipeline.txt"

PIPELINE=(gst-launch-1.0 -e)
append_source

case "$CODEC" in
  raw)
    PIPELINE+=(
      ! rtpvrawpay pt="$PAYLOAD" mtu="$MTU"
    )
    ;;
  mjpeg)
    PIPELINE+=(
      ! jpegenc quality="$JPEG_QUALITY"
      ! rtpjpegpay pt=26 mtu="$MTU"
    )
    ;;
  h264)
    encoder="$(select_h264_encoder)"
    if [[ "$encoder" == nvv4l2h264enc ]]; then
      read -r -a encoder_extra <<< "$H264_ENCODER_EXTRA"
      PIPELINE+=(
        ! videoconvert
        ! "video/x-raw,format=NV12"
        ! nvvidconv
        ! "video/x-raw(memory:NVMM),format=NV12"
        ! nvv4l2h264enc bitrate="$BITRATE" control-rate=1 iframeinterval="$GOP"
      )
      PIPELINE+=("${encoder_extra[@]}")
      PIPELINE+=(
        ! h264parse config-interval=-1
        ! "video/x-h264,alignment=au,stream-format=byte-stream"
      )
    else
      PIPELINE+=(
        ! videoconvert
        ! "video/x-raw,format=I420"
        ! x264enc tune=zerolatency speed-preset=ultrafast bitrate="$((BITRATE / 1000))" key-int-max="$GOP" bframes=0 byte-stream=true
        ! h264parse config-interval=-1
        ! "video/x-h264,alignment=au,stream-format=byte-stream"
      )
    fi
    PIPELINE+=(
      ! rtph264pay pt="$PAYLOAD" config-interval=1 mtu="$MTU"
    )
    ;;
  h265)
    encoder="$(select_h265_encoder)"
    if [[ "$encoder" == nvv4l2h265enc ]]; then
      read -r -a encoder_extra <<< "$H265_ENCODER_EXTRA"
      PIPELINE+=(
        ! videoconvert
        ! "video/x-raw,format=NV12"
        ! nvvidconv
        ! "video/x-raw(memory:NVMM),format=NV12"
        ! nvv4l2h265enc bitrate="$BITRATE" control-rate=1 iframeinterval="$GOP"
      )
      PIPELINE+=("${encoder_extra[@]}")
      PIPELINE+=(
        ! h265parse config-interval=-1
        ! "video/x-h265,alignment=au,stream-format=byte-stream"
      )
    else
      PIPELINE+=(
        ! videoconvert
        ! "video/x-raw,format=I420"
        ! x265enc tune=zerolatency speed-preset=ultrafast bitrate="$((BITRATE / 1000))" key-int-max="$GOP" option-string=bframes=0
        ! h265parse config-interval=-1
        ! "video/x-h265,alignment=au,stream-format=byte-stream"
      )
    fi
    PIPELINE+=(
      ! rtph265pay pt="$PAYLOAD" config-interval=1 mtu="$MTU"
    )
    ;;
  *)
    die "unsupported CODEC: $CODEC"
    ;;
esac

append_udp_sink

{
  printf 'HOST=%s\nPORT=%s\nCODEC=%s\nWIDTH=%s\nHEIGHT=%s\nFPS=%s\n' \
    "$HOST" "$PORT" "$CODEC" "$WIDTH" "$HEIGHT" "$FPS"
  printf 'Command:\n'
  printf '%q ' "${PIPELINE[@]}"
  printf '\n'
} > "$pipeline_file"

echo "Sender pipeline:"
printf '  %q' "${PIPELINE[@]}"
echo
echo "Log: $log_file"

"${PIPELINE[@]}" 2>&1 | tee "$log_file"

