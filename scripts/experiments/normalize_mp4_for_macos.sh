#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: normalize_mp4_for_macos.sh VIDEO.mp4 [VIDEO.mp4 ...]

Create a sibling *.macos.mp4 encoded as H.264/yuv420p with a fast-start MP4
header. The source video is kept unchanged.
EOF
}

if (($# == 0)); then
  usage >&2
  exit 2
fi

command -v gst-launch-1.0 >/dev/null \
  || { echo "gst-launch-1.0 is required" >&2; exit 1; }
gst-inspect-1.0 x264enc >/dev/null 2>&1 \
  || { echo "GStreamer x264enc is required (gstreamer1.0-plugins-ugly)" >&2; exit 1; }

for input in "$@"; do
  [[ -s "$input" ]] || { echo "Missing or empty video: $input" >&2; exit 1; }
  input="$(realpath "$input")"
  output="${input%.mp4}.macos.mp4"
  uri="$(python3 -c 'import pathlib, sys; print(pathlib.Path(sys.argv[1]).as_uri())' "$input")"
  temporary="${output}.partial"
  trap 'rm -f -- "$temporary"' EXIT

  echo "Converting: $input"
  gst-launch-1.0 -q -e \
    uridecodebin "uri=$uri" \
    ! queue \
    ! videoconvert \
    ! video/x-raw,format=I420 \
    ! x264enc speed-preset=medium bitrate=8000 key-int-max=120 \
    ! video/x-h264,profile=high \
    ! h264parse \
    ! mp4mux faststart=true \
    ! filesink "location=$temporary"

  [[ -s "$temporary" ]] || { echo "Conversion produced no video: $input" >&2; exit 1; }
  mv -f -- "$temporary" "$output"
  trap - EXIT
  echo "Created: $output"
done
