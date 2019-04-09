#! /usr/bin/env bash

speech_source=$(rospack find speech_recognition)

PATH="$speech_source"/tools/modeling${PATH:+:${PATH}}
export PATH
