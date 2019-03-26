#! /usr/bin/env bash
gst-launch-1.0 -e pulsesrc ! audioconvert ! \
    audio/x-raw, rate=16000, channels=1 ! \
    wavenc ! filesink location=test.wav
