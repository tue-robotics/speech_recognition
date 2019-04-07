#! /usr/bin/env python

from speech_recognition.kaldi_grammar import Grammar, print_graphviz

if __name__ == "__main__":
    import sys
    import os

    try:
        grammar_file = sys.argv[1]
        target = sys.argv[2]
    except:
        grammar_file = 'current_grammar.fcfg'
        target = 'T'

    dummy_model_path = os.path.realpath(__file__)

    k = Grammar(dummy_model_path, grammar_file, target)
    root_node = k.expand_tree()

    print_graphviz(root_node)

