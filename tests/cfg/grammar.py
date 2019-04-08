#! /usr/bin/env python

# Make python 2/3 compatible
from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from builtins import *

from speech_recognition.kaldi_grammar import Grammar

if __name__ == "__main__":
    import sys
    import os
    import subprocess

    try:
        grammar_file = sys.argv[1]
        target = sys.argv[2]
    except:
        grammar_file = 'current_grammar.fcfg'
        target = 'T'

    dummy_model_path = os.path.dirname(os.path.realpath(__file__))

    k = Grammar(dummy_model_path, grammar_file, target)
    k.print_graphviz()

    subprocess_exit_status = subprocess.call(["mkdynamicgrammar.bash",
        k.model_path, k.model_path_tmp])

    if subprocess_exit_status == 1:
        raise Exception("Subprocess error")

    test_sentence_1 = "bring me the coke"
    test_sentence_2 = "bring the coke to the kitchen"

    s1 = k.parse(test_sentence_1)
    s2 = k.parse(test_sentence_2)

    print("Parser output for '{}': ".format(test_sentence_1), s1)
    print("Parser output for '{}': ".format(test_sentence_2), s2)

