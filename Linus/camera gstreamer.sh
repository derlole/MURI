sh datei erstellen

#!/bin/bash

# Usage: ./start_videostream.sh [receiver_ip] [port]
RECEIVER_IP=${1:-172.18.174.218}
PORT=${2:-5600}
DEVICE="/dev/video0"

echo "Using camera: $DEVICE"
echo "Streaming to ${RECEIVER_IP}:${PORT}..."

gst-launch-1.0 v4l2src device=$DEVICE \
  ! video/x-raw,width=640,height=480,framerate=30/1 \
  ! videoconvert \
  ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast \
  ! rtph264pay config-interval=1 pt=96 \
  ! udpsink host=$RECEIVER_IP port=$PORT sync=false async=false


./start_videostream.sh <PC_IP> 5600



auf PC aufrufen (GStreamer muss installiert sein):

gst-launch-1.0 udpsrc port=5600 caps="application/x-rtp, encoding-name=H264, payload=96" \
  ! rtph264depay \
  ! avdec_h264 \
  ! videoconvert \
  ! autovideosink sync=false
