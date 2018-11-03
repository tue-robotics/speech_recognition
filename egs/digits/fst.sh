#! /usr/bin/env bash
#
# Script to prepare the Language Model FST

# Set directory paths
eg_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
data_dir=$eg_dir/data
lang_dir=$data_dir/lang
tmp_dir=$data_dir/local/tmp

# Create directory to store temp files
mkdir -p $tmp_dir

utils/prepare_lang.sh $data_dir/local/dict '!SIL' $data_dir/local/lang $data_dir/lang || exit 1;

cat $lang_dir/words.txt |awk '{print $1}' | grep -v -e "<eps>" -e "\!SIL" -e "</s>" -e "<s>" -e "sil" > $tmp_dir/words1.txt

python local/generate_bigram.py $tmp_dir/words1.txt > $tmp_dir/wp_gram.txt

local/make_rm_lm.pl $tmp_dir/wp_gram.txt > $tmp_dir/G.txt

fstcompile --isymbols=$lang_dir/words.txt --osymbols=$lang_dir/words.txt \
   --keep_isymbols=false --keep_osymbols=false $tmp_dir/G.txt > $lang_dir/G.fst

utils/validate_lang.pl $lang_dir # Note; this actually does report errors,

