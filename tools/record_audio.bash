#! /usr/bin/env bash
gst-launch-1.0 -e pulsesrc ! audioconvert ! wavenc ! filesink location=test.wav
