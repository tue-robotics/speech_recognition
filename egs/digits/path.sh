#! /usr/bin/env bash
#
# Script to set the environment variables to be used for preparing the model

# Add Kaldi libraries to path
# @TODO: Make these a part of setup.bash in KALDI repo
export PATH=$PWD/utils/:$KALDI_ROOT/src/bin:$KALDI_ROOT/tools/openfst/bin:$KALDI_ROOT/src/fstbin/:$KALDI_ROOT/src/gmmbin/:$KALDI_ROOT/src/featbin/:$KALDI_ROOT/src/lm/:$KALDI_ROOT/src/lmbin/:$KALDI_ROOT/src/sgmmbin/:$KALDI_ROOT/src/sgmm2bin/:$KALDI_ROOT/src/fgmmbin/:$KALDI_ROOT/src/latbin/:$KALDI_ROOT/src/nnet2bin/:$KALDI_ROOT/src/nnetbin:$KALDI_ROOT/src/onlinebin:$PWD:$PATH

# Export audio data directory
digits_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export DATA_ROOT="$digits_dir/wav"

# Source tools/env.sh to activate LM tools
source $KALDI_ROOT/tools/env.sh

# Set string formatting type
export LC_ALL=C
