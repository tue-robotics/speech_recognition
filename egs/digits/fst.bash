#! /usr/bin/env bash
#
# Script to prepare the Language Model FST

# Set directory paths
eg_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
data_dir=$eg_dir/data
train_=$data_dir/train
test_=$data_dir/test
lang_=$data_dir/lang
tmp_=$data_dir/local/tmp
lang_l_=$data_dir/local/lang
dict_=$data_dir/local/dict

# Remove any existing build files
rm -rf exp mfcc $train_/spk2utt $train_/cmvn.scp $train_/feats.scp $train_/split1 $test_/spk2utt $test_/cmvn.scp $test_/feats.scp $test_/split1 $lang_l_ $lang_ $tmp_ $dict_/lexiconp.txt

# Create directory to store temp files
mkdir -p $tmp_

utils/prepare_lang.sh $dict_ '!SIL' $lang_l_ $lang_ || exit 1;

cat $lang_/words.txt |awk '{print $1}' | grep -v -e "<eps>" -e "\!SIL" -e "</s>" -e "<s>" -e "sil" > $tmp_/words1.txt

python local/generate_bigram.py $tmp_/words1.txt > $tmp_/wp_gram.txt

local/make_rm_lm.pl $tmp_/wp_gram.txt > $tmp_/G.txt

echo -e "\e[35m\e[1m Done till here \e[0m"
fstcompile --isymbols=$lang_/words.txt --osymbols=$lang_/words.txt \
   --keep_isymbols=false --keep_osymbols=false $tmp_/G.txt > $lang_/G.fst

utils/validate_lang.pl $lang_ # Note; this actually does report errors,

