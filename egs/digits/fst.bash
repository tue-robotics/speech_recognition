#! /usr/bin/env bash
#
# Script to prepare the Language Model FST

# Script tuning parameters
# Language model order
lm_order=1

# Set directory paths
eg_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Directories that must exist before running this script
data_dir=$eg_dir/data
train_=$data_dir/train
test_=$data_dir/test
local_=$data_dir/local
dict_=$local_/dict

# Directories this script creates
tmp_=$local_/tmp
lang_=$data_dir/lang
lang_l_=$local_/lang

# Create symlinks to steps and utils from kaldi/egs/wsj/s5
if [ ! -d steps ]
then
    ln -s $KALDI_ROOT/egs/wsj/s5/steps .
fi

if [ ! -d utils ]
then
    ln -s $KALDI_ROOT/egs/wsj/s5/utils .
fi

# Remove any existing build files
rm -rf exp mfcc $train_/spk2utt $train_/cmvn.scp $train_/feats.scp $train_/split1 $test_/spk2utt $test_/cmvn.scp $test_/feats.scp $test_/split1 $lang_l_ $lang_ $tmp_ $dict_/lexiconp.txt

# Create directory to store temp files
mkdir -p $tmp_

utils/prepare_lang.sh $dict_ '!SIL' $lang_l_ $lang_ || exit 1

# Check for the existence of SRILM
# TODO: Replace this with IRSTLM to keep FOSS
loc=`which ngram-count`

# If loc does not return anything, check if the correct path for SRILM is in the
# environment
if [ -z $loc ]
then
    if uname -a | grep 64 >/dev/null
    then
        sdir=$KALDI_ROOT/tools/srilm/bin/i686-m64
    else
        sdir=$KALDI_ROOT/tools/srilm/bin/i686
    fi
    # Check if ngram-count exists in the updated path, if not then exit
    if [ -f $sdir/ngram-count ]
    then
        echo "Using SRILM language modelling tool from $sdir"
        export PATH=$PATH:$sdir
    else
        echo "SRILM toolkit is probably not installed. Instructions: tools/install_srilm.sh"
        exit 1
    fi
fi

ngram-count -order $lm_order -write-vocab $tmp_/vocab-full.txt -wbdiscount -text $local_/corpus.txt -lm $tmp_/lm.arpa

echo
echo "===== MAKING G.fst ====="
echo
arpa2fst --disambig-symbol=#0 --read-symbol-table=$lang_/words.txt $tmp_/lm.arpa $lang_/G.fst

