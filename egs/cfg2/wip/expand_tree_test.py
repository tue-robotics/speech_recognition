#! /usr/bin/env ipython
from kaldi_grammar import KaldiGrammar
k = KaldiGrammar('current_grammar.fcfg','T')
# words = k.expand_tree()
recognised_word = 'say'
filter = k.check_word(recognised_word)