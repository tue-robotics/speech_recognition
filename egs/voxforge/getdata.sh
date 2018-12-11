#!/bin/bash

# Copyright 2012 Vassil Panayotov
# Apache 2.0

# Downloads and extracts the data from VoxForge website

# defines the "DATA_ROOT" variable - the location to store data 
source ./path.sh

DATA_SRC="https://s3.us-east-2.amazonaws.com/common-voice-data-download/voxforge_corpus_v1.0.0.tar.gz"
DATA_TGZ=${DATA_ROOT}
DATA_EXTRACT=${DATA_ROOT}/extracted

# Create symlinks to steps and utils from kaldi/egs/wsj/s5
if [ ! -d steps ]
then
    ln -s $KALDI_ROOT/egs/wsj/s5/steps .
fi

if [ ! -d utils ]
then
    ln -s $KALDI_ROOT/egs/wsj/s5/utils .
fi

source utils/parse_options.sh

mkdir -p ${DATA_TGZ} 2>/dev/null

# Check if the executables needed for this script are present in the system
command -v wget >/dev/null 2>&1 ||\
 { echo "\"wget\" is needed but not found"'!'; exit 1; }

#echo "--- Starting VoxForge data download (may take some time) ..."
#wget -P ${DATA_TGZ} -l 1 -N -nd -c -e robots=off -A tgz -r -np ${DATA_SRC} || \
# { echo "WGET error"'!' ; exit 1 ; }
 
mkdir -p ${DATA_EXTRACT}

echo "--- Starting VoxForge archives extraction ..."
for a in ${DATA_TGZ}/*.tgz; do
  tar -C ${DATA_EXTRACT} -xf $a
done
