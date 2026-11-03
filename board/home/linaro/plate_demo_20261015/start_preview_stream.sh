#!/bin/sh
# Stage2 separate entry. Real capture dimensions and equal HDMI quadrants.
set -eu
script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
test -x "$script_dir/pcie_preview_stream"
preview_tmp=$(mktemp -d)
reader_pid=
sink_pid=
cleanup() {
    trap '' INT TERM HUP
    if [ -n "$reader_pid" ]; then kill -TERM "$reader_pid" 2>/dev/null || true; fi
    if [ -n "$sink_pid" ]; then kill -TERM "$sink_pid" 2>/dev/null || true; fi
    if [ -n "$reader_pid" ]; then wait "$reader_pid" 2>/dev/null || true; fi
    if [ -n "$sink_pid" ]; then wait "$sink_pid" 2>/dev/null || true; fi
    rm -f "$preview_tmp/frames"
    rmdir "$preview_tmp"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP
export DISPLAY=${DISPLAY:-:0}
mkfifo "$preview_tmp/frames"
if [ -r /dev/plate_video_dma ] && [ -w /dev/plate_video_dma ]; then
    "$script_dir/pcie_preview_stream" "$@" --stdout-rgb565 > "$preview_tmp/frames" &
else
    sudo -n "$script_dir/pcie_preview_stream" "$@" --stdout-rgb565 > "$preview_tmp/frames" &
fi
reader_pid=$!
gst-launch-1.0 -q fdsrc fd=0 blocksize=1843200 \
    ! rawvideoparse format=rgb16 width=1280 height=720 framerate=30/1 \
    ! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream \
    ! videoconvert ! videoscale \
    ! video/x-raw,width=928,height=522,pixel-aspect-ratio=1/1 \
    ! "${VIDEO_SINK:-ximagesink}" sync=false handle-events=false \
    < "$preview_tmp/frames" &
sink_pid=$!
sink_status=0
wait "$sink_pid" || sink_status=$?
# A closed sink must request the reader's ordered STOP, not leave it blocked.
kill -TERM "$reader_pid" 2>/dev/null || true
reader_status=0
wait "$reader_pid" || reader_status=$?
reader_pid=
sink_pid=
if [ "$reader_status" -ne 0 ]; then exit "$reader_status"; fi
exit "$sink_status"
