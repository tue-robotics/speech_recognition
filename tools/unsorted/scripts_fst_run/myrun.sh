#!/bin/bash

# Copyright 2013  Bagher BabaAli

. ./cmd.sh 
[ -f path.sh ] && . ./path.sh

# Acoustic model parameters
numLeavesTri1=450     # 800
numGaussTri1=6*450    # 13200
numLeavesMLLT=450     # 800
numGaussMLLT=6*450    # 13200
numLeavesSAT=450      # 800
numGaussSAT=6*450     # 13200


numGaussUBM=64  # 200, 256
numLeavesSGMM=450   # 1000
numGaussSGMM=6*450    # 16000

decode_nj=8
train_nj=8
train_cmd=run.pl
decode_cmd=run.pl
#train_cmd=queue.pl
#decode_cmd=queue.pl
#mfcc=0
#mono=0
#tri1=0

mfcc=0
mono=0
tri1=1

if [ $mfcc -eq 1 ]; then

echo ============================================================================
echo "         MFCC Feature Extration & CMVN for Training and Test set           "
echo ============================================================================


# Now make MFCC features.
mfccdir=mfcc
use_pitch=false
use_ffv=false

for x in train test ; do  #train

	steps/make_mfcc.sh --cmd "$train_cmd" --nj 8 data/$x exp/make_mfcc/$x $mfccdir || exit 1;
	steps/compute_cmvn_stats.sh data/$x exp/make_mfcc/$x $mfccdir || exit 1;
done

fi

if [ $mono -eq 1 ]; then

echo ============================================================================
echo "                     MonoPhone Training & Decoding                        "
echo ============================================================================

steps/train_mono.sh  --nj "$train_nj" --cmd "$train_cmd" data/train data/lang exp/mono || exit 1;

utils/mkgraph.sh --mono data/lang exp/mono exp/mono/graph || exit 1;

steps/decode.sh --nj "$decode_nj" --cmd "$decode_cmd" \
	exp/mono/graph data/test exp/mono/decode || exit 1;
fi

if [ $tri1 -eq 1 ]; then

echo ============================================================================
echo "           tri1 : Deltas + Delta-Deltas Training & Decoding               "
echo ============================================================================

steps/align_si.sh --boost-silence 1.25 --nj "$train_nj" --cmd "$train_cmd" data/train data/lang exp/mono exp/mono_ali || exit 1;

#
## Train tri1, which is deltas + delta-deltas, on train data.
#
for sen in 450; do
for gauss in 4; do
gauss=$(($sen * $gauss))
steps/train_deltas.sh --cmd "$train_cmd" $sen $gauss data/train data/lang exp/mono_ali exp/tri1_${sen}_${gauss} || exit 1;

utils/mkgraph.sh data/lang exp/tri1_${sen}_${gauss} exp/tri1_${sen}_${gauss}/graph || exit 1;

steps/decode.sh --nj "$decode_nj" --cmd "$decode_cmd" \
 exp/tri1_${sen}_${gauss}/graph data/test exp/tri1_${sen}_${gauss}/decode || exit 1;


done
done
fi

#<<"over"
echo ============================================================================
echo "                 tri2 : LDA + MLLT Training & Decoding                    "
echo ============================================================================


for sen in 450; do
for gauss in 4; do
gauss=$(($sen * $gauss))

if [ ! -f exp/tri1_${sen}_${gauss}_ali/final.mdl ]; then
steps/align_si.sh --nj "$train_nj" --cmd "$train_cmd" \
  data/train data/lang exp/tri1_${sen}_${gauss} exp/tri1_${sen}_${gauss}_ali || exit 1;
fi

if [ ! -f exp/tri2_${sen}_${gauss}/final.mdl ]; then
steps/train_lda_mllt.sh --cmd "$train_cmd" \
 --splice-opts "--left-context=3 --right-context=3" \
 $sen $gauss data/train data/lang exp/tri1_${sen}_${gauss}_ali exp/tri2_${sen}_${gauss} # || exit 1;
fi

if [ ! -f exp/tri2_${sen}_${gauss}/graph/words.txt ]; then
utils/mkgraph.sh data/lang exp/tri2_${sen}_${gauss} exp/tri2_${sen}_${gauss}/graph || exit 1;
fi

if [ ! -f exp/tri2_${sen}_${gauss}/decode/wer_10 ]; then
steps/decode.sh --nj "$decode_nj" --cmd "$decode_cmd" \
 exp/tri2_${sen}_${gauss}/graph data/test exp/tri2_${sen}_${gauss}/decode || exit 1;
fi

#Compute results
for x in exp/*/decode*; do
  [ -d $x ] && grep WER $x/wer_* | utils/best_wer.sh >> Results.txt
done

done
done


<<"over"
echo ============================================================================
echo "              tri3 : LDA + MLLT + SAT Training & Decoding                 "
echo ============================================================================

#mkdir -p exp/tri2
#cp -r exp/tri2_650_2600 exp/tri2 ## best model

# Align tri2 system with train data.
steps/align_si.sh --nj "$train_nj" --cmd "$train_cmd" \
 --use-graphs true data/train data/lang exp/tri2 exp/tri2_ali || exit 1;


for sen in 450 500 550 600 650 700 750; do
for gauss in 4 6 8; do
gauss=$(($sen * $gauss))


# From tri2 system, train tri3 which is LDA + MLLT + SAT.
steps/train_sat.sh --cmd "$train_cmd" \
 $sen $gauss data/train data/lang exp/tri2_ali exp/tri3_${sen}_${gauss} || exit 1;

utils/mkgraph.sh data/lang exp/tri3_${sen}_${gauss} exp/tri3_${sen}_${gauss}/graph || exit 1;

##steps/decode_fmllr.sh --nj "$decode_nj" --cmd "$decode_cmd" \
##  exp/tri3/graph data/dev exp/tri3/decode_dev || exit 1;

steps/decode.sh --nj "$decode_nj" --cmd "$decode_cmd" \
 exp/tri3_${sen}_${gauss}/graph data/test exp/tri3_${sen}_${gauss}/decode || exit 1;
#ea
done
done


echo ============================================================================
echo "                        SGMM2 Training & Decoding                         "
echo ============================================================================

#steps/align_fmllr.sh --nj "$train_nj" --cmd "$train_cmd" \
# data/train data/lang exp/tri2 exp/tri2_ali || exit 1;

#steps/decode_fmllr.sh --nj "$decode_nj" --cmd "$decode_cmd" \
# exp/tri2/graph data/test exp/tri2/decode || exit 1;

steps/align_si.sh --nj "$train_nj" --cmd "$train_cmd" \
 --use-graphs true data/train data/lang exp/tri2 exp/tri2_ali || exit 1;   # copy the best model from tri2_1350_24300/21600 to tri2
<<over
steps/train_ubm.sh --cmd "$train_cmd" \
 $numGaussUBM data/train data/lang exp/tri2_ali exp/ubm4_t2a || exit 1;

bash -x steps/train_sgmm.sh --cmd "$train_cmd" $numLeavesSGMM $numGaussSGMM \
 data/train data/lang exp/tri2_ali exp/ubm4_t2a/final.ubm exp/sgmm || exit 1;

#bash -x steps/train_sgmm2.sh --cmd "$train_cmd" $numLeavesSGMM $numGaussSGMM \
 #data/train data/lang exp/tri2_ali exp/ubm4_t2a/final.ubm exp/sgmm2_4_t2a || exit 1;

utils/mkgraph.sh data/lang exp/sgmm2_4_t2a exp/sgmm/graph || exit 1;

#utils/mkgraph.sh data/lang exp/sgmm2_4_t2a exp/sgmm2_4_t2a/graph || exit 1;


## steps/decode_sgmm2.sh --nj "$decode_nj" --cmd "$decode_cmd"\
## --transform-dir exp/tri3/decode_dev exp/sgmm2_4/graph 22hrdata/dev \
## exp/sgmm2_4/decode_dev || exit 1;

steps/decode_sgmm.sh --nj `cat exp/tri2/decode/num_jobs`  --cmd "$decode_cmd"\
  exp/sgmm/graph data/test \
 exp/sgmm/decode || exit 1;

#steps/decode_sgmm2.sh --nj `cat exp/tri2/decode/num_jobs`  --cmd "$decode_cmd"\
 # exp/sgmm2_4_t2a/graph data/test \
 #exp/sgmm2_4_t2a/decode || exit 1;

#ea
#over

#<<"over"

echo " ============================================================================ "
echo "                    MMI + SGMM2 Training & Decoding                       "
echo " ============================================================================ "

steps/align_fmllr.sh --nj "$train_nj" --cmd "$train_cmd" \
 data/train data/lang exp/tri2 exp/tri2_ali || exit 1;


steps/align_sgmm2.sh --nj 10 --cmd "$train_cmd" \
 --transform-dir exp/tri2_ali --use-graphs true --use-gselect true data/train \
 data/lang exp/sgmm2_4_t2a exp/sgmm2_4_ali_b_t2a || exit 1;

steps/make_denlats_sgmm2.sh --nj "$train_nj" --sub-split "$train_nj" --cmd "$decode_cmd"\
 --transform-dir exp/tri2_ali data/train 22hrdata/lang exp/sgmm2_4_ali_b_t2a \
 exp/sgmm2_4_denlats_b_t2a || exit 1;

# steps/train_mmi_sgmm2.sh --cmd "$decode_cmd" \
# --transform-dir exp/tri3_ali --boost 0.x --zero-if-disjoint true \
# data/train_full data/lang exp/sgmm2_4_ali exp/sgmm2_4_denlats \
# exp/sgmm2_4_mmi_b0.x || exit 1;


steps/train_mmi_sgmm2.sh --cmd "$decode_cmd" \
 --transform-dir exp/tri2_ali --boost 0.1 \
 data/train data/lang exp/sgmm2_4_ali_b_t2a exp/sgmm2_4_denlats_b_t2a \
 exp/sgmm2_4_mmi_b0.1_t2a || exit 1;



for iter in 1 2 3 4 5; do
##  steps/decode_sgmm2_rescore.sh --cmd "$decode_cmd" --iter $iter \
##   --transform-dir exp/tri3/decode_dev data/lang_bg 22hrdata/dev \
##   exp/sgmm2_4/decode_dev exp/sgmm2_4_mmi_b0.1/decode_dev_it$iter || exit 1;

  steps/decode_sgmm2_rescore.sh --cmd "$decode_cmd" --iter $iter \
   --transform-dir exp/tri2/decode data/lang 22hrdata/test \
   exp/sgmm2_4_t2a/decode exp/sgmm2_4_mmi_b0.1_t2a/decode_it$iter || exit 1;
done

#ea

#<<"over"


echo ============================================================================
echo "                    DNN Hybrid Training & Decoding                        "
echo ============================================================================

# DNN hybrid system training parameters
dnn_mem_reqs="mem_free=1.0G,ram_free=0.2G"
dnn_extra_opts="--num_epochs 20 --num-epochs-extra 10 --add-layers-period 1 --shrink-interval 3"

#steps/align_fmllr.sh --nj "$decode_nj" --cmd "$decode_cmd" \
# data/train_full data/lang exp/tri2 exp/tri2_ali_fmllr || exit 1;


steps/train_nnet_cpu.sh --mix-up 5000 --initial-learning-rate 0.015 \
  --final-learning-rate 0.002 --num-hidden-layers 2 --num-parameters 3000000 \
  --num-jobs-nnet "$train_nj" --cmd "$train_cmd" "${dnn_train_extra_opts[@]}" \
  data/train data/lang exp/tri2_ali exp/tri4_nnet_t2a  || exit 1;


decode_extra_opts=(--num-threads 4 --parallel-opts "-pe sph 4 -l mem_free=4G,ram_free=0.7G")
  steps/decode_nnet_cpu.sh --cmd "$decode_cmd" --nj "$decode_nj" "${decode_extra_opts[@]}" \
  --transform-dir exp/tri2/decode exp/tri2/graph data/test \
  exp/tri4_nnet_t2a/decode | tee exp/tri4_nnet_t2a/decode/decode.log

#Decoding for the fMLLR on top of tri2b
#    steps/decode_fmllr.sh --config conf/decode.config --nj 4 --cmd "$train_cmd" \
#        exp/tri2/graph data/test exp/tri2/decode || exit 1;

#steps/decode_nnet_cpu.sh --cmd "$decode_cmd" --nj 20 "${decode_extra_opts[@]}" \
#  --transform-dir exp/tri3/decode_test exp/tri3/graph 22hrdata/test_crop \
#  exp/tri4_nnet/decode_test | tee exp/tri4_nnet/decode_test/decode.log


exit

#<<"over"

echo ============================================================================
echo "                    System Combination (DNN+SGMM)                         "
echo ============================================================================

for iter in 1 2 3 4; do
##  local/score_combine.sh --cmd "$decode_cmd" \
##   data/dev data/lang_bg exp/tri4_nnet/decode_dev \
##   exp/sgmm2_4_mmi_b0.1/decode_dev_it$iter exp/combine_2/decode_dev_it$iter


  local/score_combine.sh --cmd "$decode_cmd" \
   data/test_crop data/lang exp/tri4_nnet_t2a/decode \
   exp/sgmm2_4_mmi_b0.1_t2a/decode_it$iter exp/combine_2/decode_it$iter
done
exit
echo ============================================================================
echo "                    Getting Results [see RESULTS file]                    "
echo ============================================================================

for x in exp/*/decode*; do
  [ -d $x ] && grep WER $x/wer_* | utils/best_wer.sh >> /work/kaldi/kaldi-trunk/egs/timit/s5exp/timitresults
done 
over
echo ============================================================================
echo "Finished successfully on" `date`
echo ============================================================================
#over
exit 0
