#! /usr/bin/env ipython
from kaldi_grammar import KaldiGrammar, expand_tree, print_tree
k = KaldiGrammar('current_grammar.fcfg','T')
# words = k.expand_tree()
# recognised_word = 'say'
# filter = k.check_word(recognised_word)
# auto = k.autocomplete()
root_node = expand_tree(k.parser.rules)
print_tree(root_node)