#! /usr/bin/env bash

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
