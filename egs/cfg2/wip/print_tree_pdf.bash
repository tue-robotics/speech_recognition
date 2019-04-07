#! /usr/bin/env bash

./kaldi_grammar.py > tree.dot
dot -Tpdf tree.dot > tree.pdf
rm tree.dot
