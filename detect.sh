#!/bin/sh
cd /home/rrc/Ross/darknet_ab/
rm webcamshot.jpg
fswebcam -r 416x416 --jpeg 100 -D 2 webcamshot.jpg
./darknet detect cfg/yolov3-tiny.cfg yolov3-tiny.weights webcamshot.jpg
rm detecting.temp