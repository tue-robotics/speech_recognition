#! /usr/bin/env python
import os
from grammar_parser.cfgparser import CFGParser


class KaldiGrammar:
    """
    Class KaldiGrammar uses as input a grammar file with extension '.fcfg' and has two functions:
    get_rule_element: extracts the defined grammar rules
    get_words: extracts the unique words and creates 'corpus.txt' which is used to build 'G.fst'
    """
    def __init__(self, grammar_file_string, target):

        if os.path.exists(grammar_file_string):
            self.parser = CFGParser.fromfile(grammar_file_string)
            self.grammar_file = grammar_file_string
        else:
            self.parser = CFGParser.fromstring(grammar_file_string)
            self.grammar_string = grammar_file_string

        self.target = target

    # ----------------------------------------------------------------------------------------------------


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

        words = [word.upper() + "\n" for word in list(words)]

        # Create corpus.txt file and save the words list
        with open("corpus.txt", "w") as f:
            for word in words:
                f.write(word)

