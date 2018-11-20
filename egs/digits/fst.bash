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
wav_=$eg_dir/wav
wav_all_=$eg_dir/all

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

# Check if the audio files directory exists
if [ ! -d $wav_ -o ! -d $wav_all_ ]
then
    7z x audio_files.7z
fi

# Remove any existing build files
rm -rf exp mfcc $train_/wav.scp $train_/spk2utt $train_/cmvn.scp \
$train_/feats.scp $train_/split1 $test_/wav.scp $test_/spk2utt $test_/cmvn.scp \
$test_/feats.scp $test_/split1 $lang_l_ $lang_ $tmp_ $dict_/lexiconp.txt

# Create directory to store temp files
mkdir -p $tmp_

echo
echo -e "\e[35m\e[1m==== Preparing Lexicon (L.fst) ====\e[0m"
echo

# Make L.fst
utils/prepare_lang.sh $dict_ '!SIL' $lang_l_ $lang_ || exit 1

echo
echo -e "\e[35m\e[1m==== Preparing Language Model (G.fst) ====\e[0m"
echo

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
        echo "SRILM toolkit is probably not installed. Instructions: $KALDI_ROOT/tools/install_srilm.sh"
        exit 1
    fi
fi

# Create LM in ARPA format
ngram-count -order $lm_order -write-vocab $tmp_/vocab-full.txt -wbdiscount -text $local_/corpus.txt -lm $tmp_/lm.arpa

# Make G.fst
arpa2fst --disambig-symbol=#0 --read-symbol-table=$lang_/words.txt $tmp_/lm.arpa $lang_/G.fst

# Checking that G is stochastic [note, it wouldn't be for an Arpa]
fstisstochastic $lang_/G.fst || echo Error: G is not stochastic

# Checking that G.fst is determinizable.
fstdeterminize $lang_/G.fst /dev/null || echo Error determinizing G.

# Checking that L_disambig.fst is determinizable.
fstdeterminize $lang_/L_disambig.fst /dev/null || echo Error determinizing L.

# Checking that disambiguated lexicon times G is determinizable
fsttablecompose $lang_/L_disambig.fst $lang_/G.fst | \
   fstdeterminize >/dev/null || echo Error

# Checking that LG is stochastic:
fsttablecompose $lang_/L.fst $lang_/G.fst | \
   fstisstochastic || echo Error: LG is not stochastic.

# Checking that L_disambig.G is stochastic:
fsttablecompose $lang_/L_disambig.fst $lang_/G.fst | \
   fstisstochastic || echo Error: LG is not stochastic.

# Validate lang directory
utils/validate_lang.pl $lang_ # Note; this actually does report errors,

