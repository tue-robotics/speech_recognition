#!/bin/bash

. path.sh

mkdir -p $PWD/data/lang

lang=$PWD/data/lang

# Create the phone bigram LM
utils/prepare_lang.sh data/local/dict '!SIL' data/local/lang $lang

python gen_wnet.py data/local/plain-text/text_c1 "sil" $lang/G.txt

fstcompile --isymbols=$lang/words.txt --osymbols=$lang/words.txt $lang/G.txt | fstdeterminizestar | fstminimize > $lang/G.fst

# to view any fst in graph form
#fstdraw --isymbols=$lang/words.txt --osymbols=$lang/words.txt $lang/G.fst  | dot -Tpng > out.png
