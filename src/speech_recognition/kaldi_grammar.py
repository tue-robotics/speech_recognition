#! /usr/bin/env python

# Make python 2/3 compatible
from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from builtins import *

import os
import shutil
from grammar_parser.cfgparser import CFGParser


class KaldiGrammar:
    """
    Class KaldiGrammar uses as input a grammar file with extension '.fcfg' and has two functions:
    get_rule_element: extracts the defined grammar rules
    get_words: extracts the unique words and creates 'corpus.txt' which is used to build 'G.fst'
    """
    def __init__(self, grammar_file_string, target, model_path):

        if os.path.exists(grammar_file_string):
            self.parser = CFGParser.fromfile(grammar_file_string)
            self.grammar_file = grammar_file_string
        else:
            self.parser = CFGParser.fromstring(grammar_file_string)
            self.grammar_string = grammar_file_string

        self.target = target
        self.model_path = model_path
        self.model_path_tmp = os.path.join(self.model_path, "tmp")

        if not os.path.exists(self.model_path):
            raise Exception("Model path '{}' does not exist".format(self.model_path))
        else:
            if os.path.exists(self.model_path_tmp):
                shutil.rmtree(self.model_path_tmp)

            os.mkdir(self.model_path_tmp)


    def get_words(self):
        """
        Extracts list with all the unique words, used within the grammar and
        create file 'corpus.txt' which is used to build 'G.fst'
        """

        # Extract rules from the grammar file
        rules = self.parser.rules

        # Extract words
        words = set()

        for key, value in rules.iteritems():
            # Get the list of options for the rule value
            options = value.options
            for option in options:
                # Get the list of conjuncts for option 'option'
                conjuncts = option.conjuncts
                for conjunct in conjuncts:
                    # If conjunct is not a variable put its value in the set of words
                    if not conjunct.is_variable:
                        words.add(conjunct.name)

        words = [word.upper() for word in list(words)]
        words.sort()

        # Create corpus.txt file and save the words list
        corpus_path = os.path.join(self.model_path_tmp, "corpus.txt")
        with open(corpus_path, "w") as f:
            for word in words:
                f.write(word + "\n")

