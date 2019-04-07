#! /usr/bin/env bash

if [ -z "$1" ]
then
    echo "Path to corpus.txt not specified"
    exit 1
fi

if [ ! -f "$1/corpus.txt" ]
then
    echo "corpus.txt does not exist at '$1/corpus.txt'"
    exit 1
fi

loctmp="$1"

# Language model order
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
  -text $loctmp/corpus.txt -lm $loctmp/lm.arpa

