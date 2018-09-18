export PATH=$PWD/utils/:$KALDI_ROOT/src/bin:$KALDI_ROOT/tools/openfst/bin:$KALDI_ROOT/src/fstbin/:$KALDI_ROOT/src/gmmbin/:$KALDI_ROOT/src/featbin/:$KALDI_ROOT/src/lm/:$KALDI_ROOT/src/sgmmbin/:$KALDI_ROOT/src/sgmm2bin/:$KALDI_ROOT/src/fgmmbin/:$KALDI_ROOT/src/latbin/:$KALDI_ROOT/src/nnet2bin/:$KALDI_ROOT/src/nnetbin:$KALDI_ROOT/src/onlinebin:$PWD:$PATH
digits_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export DATA_ROOT="$digits_dir/wav"
source $KALDI_ROOT/tools/env.sh
export LC_ALL=C
