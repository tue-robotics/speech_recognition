#! /usr/bin/env bash

if [ -z "$1" ]
then
    echo "Model path not specified"
    exit 1
fi

model_path="$1"

if [ -z "$2" ]
then
    echo "Path to 'corpus.txt' not specified"
    exit 1
fi
model_path_tmp="$2"

if [ ! -f "$model_path_tmp"/corpus.txt ]
then
    echo "corpus.txt does not exist at '$model_path_tmp/corpus.txt'"
    exit 1
fi

mkdynamicgrammar.bash "$model_path" "$model_path_tmp"

mkgraph.sh "$model_path_tmp" "$model_path" "$model_path_tmp"/graph

cp "$model_path_tmp"/graph/HCLG.fst "$model_path"
