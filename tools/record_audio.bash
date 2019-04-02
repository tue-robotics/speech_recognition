#! /usr/bin/env bash

input_src=$(pactl list | grep -A2 'Source #' | grep 'Name: ' | cut -d" " -f2)

echo -e "Audio sources found:"
echo -e "--------------------"
echo -e "$input_src"
echo
echo -e "Setting up gstreamer pipeline:"
echo -e "------------------------------"

if [ -z "$1" ]
then
    gst-launch-1.0 -e pulsesrc ! audioconvert ! \
    audio/x-raw, rate=16000, channels=1 ! \
    wavenc ! filesink location=test.wav
else
    gst-launch-1.0 -e pulsesrc device="$1" ! audioconvert ! \
    audio/x-raw, rate=16000, channels=1 ! \
    wavenc ! filesink location=test.wav
fi

echo
echo -e "Recording complete"
echo -e "------------------------------"
