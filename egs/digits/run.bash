#! /usr/bin/env bash

train_cmd="utils/run.pl"
decode_cmd="utils/run.pl"

#if [ ! -d wav ]; then
#  wget http://www.openslr.org/resources/1/waves_yesno.tar.gz || exit 1;
  # was:
  # wget http://sourceforge.net/projects/kaldi/files/waves_yesno.tar.gz || exit 1;
#  tar -xvzf waves_yesno.tar.gz || exit 1;
#fi

#train_digits=train_digits
#test_base_name=test_digits

rm -rf exp mfcc


# Feature extraction
for x in train_digits test_digits
do
    steps/make_mfcc.sh --nj 1 data/$x exp/make_mfcc/$x mfcc || exit 1
    steps/compute_cmvn_stats.sh data/$x exp/make_mfcc/$x mfcc || exit 1
    utils/fix_data_dir.sh data/$x
done

echo "Features Extracted Successfully."

# Mono training
steps/train_mono.sh --nj 1 --cmd "$train_cmd" --totgauss 1600 data/train_digits data/lang exp/mono0a || exit 1

echo "Mono Training Complete."

# Graph compilation
utils/mkgraph.sh --mono data/lang exp/mono0a exp/mono0a/graph_tgpr || exit 1

# Decoding
#steps/decode.sh --nj 1 --cmd "$decode_cmd" \
 #   exp/mono0a/graph_tgpr data/test_digits exp/mono0a/decode_test_digits
steps/decode.sh --config conf/decode.config --nj 1 --cmd "$decode_cmd" exp/mono0a/graph_tgpr data/test_digits exp/mono0a/decode_test_digits || exit 1

for x in exp/*/decode*
do
    [ -d $x ] && grep WER $x/wer_* | utils/best_wer.sh
done
