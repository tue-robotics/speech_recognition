#! /usr/bin/env bash

./grammar.py > tree.dot
dot -Tpdf tree.dot > tree.pdf
rm tree.dot
