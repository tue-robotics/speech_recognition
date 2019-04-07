#! /usr/bin/env bash

if [ -z "$1" ]
then
    echo "Model path not specified"
    exit 1
fi

model_path="$1"

if [ -z "$2" ]
then
    echo "Path to 'corpus.txt' not specified"
    exit 1
fi
model_path_tmp="$2"

if [ ! -f "$model_path_tmp/corpus.txt" ]
then
    echo "corpus.txt does not exist at '$model_path_tmp/corpus.txt'"
    exit 1
fi

# -----------------------------------------------------------------------------
# Language model preparation
order=1

loc=$(which ngram-count)
if [ -z $loc ]
then
  if uname -a | grep 64 > /dev/null
  then
    # some kind of 64 bit...
    sdir=$KALDI_ROOT/tools/srilm/bin/i686-m64
  else
    sdir=$KALDI_ROOT/tools/srilm/bin/i686
  fi

  if [ -f $sdir/ngram-count ]
  then
    echo Using SRILM tools from $sdir
    PATH=$PATH:$sdir
    export PATH
  else
    echo You appear to not have SRILM tools installed, either on your path,
    echo or installed in $sdir.  See tools/install_srilm.sh for installation
    echo instructions.
    exit 1
  fi
fi

ngram-count -order $order -wbdiscount \
  -text $model_path_tmp/corpus.txt -lm $model_path_tmp/lm.arpa

# -----------------------------------------------------------------------------
# G.fst preparation

cp "$model_path"/words.txt $model_path_tmp

cat $model_path_tmp/lm.arpa | \
  arpa2fst --disambig-symbol=#0 \
           --read-symbol-table=$model_path_tmp/words.txt - $model_path_tmp/G.fst

fstisstochastic $model_path_tmp/G.fst || echo "G.fst not stochastic"


