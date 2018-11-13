#! /usr/bin/env bash

train_cmd="utils/run.pl"
decode_cmd="utils/run.pl"
nj=1

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
mfcc_=mfcc

echo
echo -e "\e[35m\e[1m==== Preparing Acoustic Data (spk2utt and data validation) ====\e[0m"
echo
# Making spk2utt files
utils/utt2spk_to_spk2utt.pl $train_/utt2spk > $train_/spk2utt
utils/utt2spk_to_spk2utt.pl $test_/utt2spk > $test_/spk2utt

# Uncomment scripts below if there are any problems with data sorting
# script for checking prepared data
# utils/validate_data_dir.sh data/train
# utils/validate_data_dir.sh data/train

# tool for proper data sorting if needed
# utils/fix_data_dir.sh data/train
# utils/fix_data_dir.sh data/train

echo
echo -e "\e[35m\e[1m==== Feature Extraction (mfcc) ====\e[0m"
echo
# Making feats.scp files
steps/make_mfcc.sh --nj $nj --cmd "$train_cmd" $train_ exp/make_mfcc/train $mfcc_
steps/make_mfcc.sh --nj $nj --cmd "$train_cmd" $test_ exp/make_mfcc/test $mfcc_

# Making cmvn.scp files
steps/compute_cmvn_stats.sh $train_ exp/make_mfcc/train $mfcc_
steps/compute_cmvn_stats.sh $test_ exp/make_mfcc/test $mfcc_

echo
echo -e "\e[35m\e[1m==== Mono Training ====\e[0m"
echo
# Make monophone model
# TODO Test without gaussians as well
steps/train_mono.sh --nj $nj --cmd "$train_cmd" --totgauss 1600 $train_ $lang_ exp/mono  || exit 1

echo
echo -e "\e[35m\e[1m==== Mono Decoding ====\e[0m"
echo
# Make decoding graph for monophone model
utils/mkgraph.sh --mono $lang_ exp/mono exp/mono/graph || exit 1
# Decode monophone model with test data
steps/decode.sh --config conf/decode.config --nj $nj --cmd "$decode_cmd" exp/mono/graph $test_ exp/mono/decode

echo
echo -e "\e[35m\e[1m==== Mono Alignment ====\e[0m"
echo
# Align phones to start triphone modeling
steps/align_si.sh --nj $nj --cmd "$train_cmd" $train_ $lang_ exp/mono exp/mono_ali || exit 1

echo
echo -e "\e[35m\e[1m==== Tri1 Training ====\e[0m"
echo
# Make first pass triphone model
steps/train_deltas.sh --cmd "$train_cmd" 2000 11000 $train_ $lang_ exp/mono_ali exp/tri1 || exit 1

echo
echo -e "\e[35m\e[1m==== Tri1 Decoding ====\e[0m"
echo
# Make decoding graph for first pass triphone model
utils/mkgraph.sh $lang_ exp/tri1 exp/tri1/graph || exit 1
# Decode first pass triphone model with test data
steps/decode.sh --config conf/decode.config --nj $nj --cmd "$decode_cmd" exp/tri1/graph $test_ exp/tri1/decode

# Get WER
for x in exp/*/decode*
do
    [ -d $x ] && grep WER $x/wer_* | utils/best_wer.sh
done

echo
echo -e "\e[35m\e[1m==== Execution completed ====\e[0m"
echo

