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

    dummy_model_path = os.path.dirname(os.path.realpath(__file__))

    k = Grammar(dummy_model_path, grammar_file, target)
    root_node = k.expand_tree()

    print_graphviz(root_node)

#    test_sentence_1 = "bring me the coke"
#    test_sentence_2 = "bring the coke to the kitchen"

#    s1 = k.parse(test_sentence_1)
#    s2 = k.parse(test_sentence_2)

#    print(s1)
#    print(s2)

