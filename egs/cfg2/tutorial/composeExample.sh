#! /usr/bin/env bash

# Based on http://www.isle.illinois.edu/sst/courses/minicourses/2009/lecture6.pdf

mkdir -p build bin
BIN=bin
BUILD=build

bash compileAndDraw.sh sent.fsa
bash compileAndDraw.sh dict.fst

fstcompose --fst_compat_symbols=false $BIN/sent.fsa $BIN/dict.fst > $BIN/strings.fst
fstdraw --portrait  $BIN/strings.fst | dot -Tsvg >  $BIN/strings.svg

echo "Done composing. Output is at $BIN/strings.svg"
echo 'Example sentences:'
echo '------------------'

for i in $(seq 1 10)
do
    fstrandgen --seed=$RANDOM $BIN/strings.fst | fstproject --project_output |
    fstprint --acceptor --isymbols="$BUILD/dict.osyms" |
    awk '{printf("%s ",$3)}END{printf("\n")}'
done
