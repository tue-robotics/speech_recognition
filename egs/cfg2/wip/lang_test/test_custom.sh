#!/bin/bash
#
# Script to test the model with single audio source file

. ./path.sh || exit 1

# The train and decode commands are defined in the file below
. ./cmd.sh || exit 1

# The number of parallel jobs to be started for some parts of the recipe
# Make sure you have enough resources(CPUs and RAM) to accomodate this number of jobs
njobs=1

# The user of this script could change some of the above parameters. Example:
# /bin/bash run.sh --njobs 2
. utils/parse_options.sh || exit 1
[[ $# -ge 1 ]] && { echo "Unexpected arguments"; exit 1; }

# Now make MFCC features.
mfccdir=${DATA_ROOT}/mfcc
mkdir -p $mfccdir

for x in test2
do
 steps/make_mfcc.sh --cmd "$train_cmd" --nj $njobs \
   data/$x exp/make_mfcc/$x $mfccdir || exit 1
 steps/compute_cmvn_stats.sh data/$x exp/make_mfcc/$x $mfccdir || exit 1
done

# decode tri2a
steps/decode.sh --config conf/decode.config --nj "$njobs" --cmd "$decode_cmd" \
  exp/tri2a/graph data/test2 exp/tri2a/decode2

echo
echo -e "\e[35m\e[1m==== Calculate Best Word Error Rate (WER) ====\e[0m"
echo
# Get WER
for x in exp/*/decode*
do
    [ -d $x ] && grep WER $x/wer_* | utils/best_wer.sh
done

echo
echo -e "\e[35m\e[1m==== Execution completed ====\e[0m"
echo

