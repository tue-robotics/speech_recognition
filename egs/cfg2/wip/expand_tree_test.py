#! /usr/bin/env ipython
from kaldi_grammar import KaldiGrammar
k = KaldiGrammar('current_grammar.fcfg','T')
words = k.expand_tree()