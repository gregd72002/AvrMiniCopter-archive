#!/bin/sh
if [ "$1" != "start" ]; then
	echo "stoping"
	killall raspivid
else
	raspivid -t 0 -b 500000 -h 480 -w 640 -fps 20 -g 20 -n -o - | gst-launch-1.0 fdsrc ! h264parse ! rtph264pay config-interval=1 pt=96 ! gdppay ! udpsink port=$3 host=$2 &
	#raspivid -n -w 1280 -h 720 -b 4500000 -fps 30 -g 20 -t 0 -o - | gst-launch-1.0 -v fdsrc !  h264parse ! tee name=splitter ! queue ! rtph264pay config-interval=10 pt=96 ! udpsink host=$2 port=$3 splitter. ! queue ! filesink location="/rpicopter/videofile.h264"
	echo "starting;"
fi

