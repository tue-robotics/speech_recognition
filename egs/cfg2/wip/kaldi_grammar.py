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

    def get_rule_element(self, depth=0):
        """
        Extracts the grammar rules, defined within the grammar file

        :param depth: level of depth of the grammar rule
        :return: grammar tree
        """
        RulesList = {}
        # print(depth)
        if self.target not in self.parser.rules:
            raise Exception("Target {} not in parser rules".format(self.target))

        # If already present in RULES, return it
        if self.target in RulesList:
            return RulesList[self.target]

        # Get the rule
        rule = self.parser.rules[self.target]

        # Iterate over all options
        option_alternative_list = []
        for opt in rule.options:

            # Iterate over all conjunctions
            conjunctions_list = []
            for conj in opt.conjuncts:
                # If the conjunction is already present
                if conj.name in RulesList:
                    conjunctions_list.append(RulesList[conj.name])
                    continue

                # If variable: go one level deeper
                if conj.is_variable:
                    result = get_rule_element(conj.name, self.parser, depth + 1)
                    if result:
                        conjunctions_list.append(result)
                else:
                    # Add a new literal to the list
                    RulesList[conj.name] = conj.name
                    conjunctions_list.append(RulesList[conj.name])

            if len(conjunctions_list) == 1:
                option_alternative_list.append(conjunctions_list[0])
            else:
                option_alternative_list.append(tuple(conjunctions_list))

        if len(option_alternative_list) == 1:
            RulesList[self.target] = option_alternative_list[0]
        else:
            RulesList[self.target] = option_alternative_list

        return RulesList[self.target]

    def get_words(self):
        """
        Extracts list with all the words, used within the grammar and
        create file 'corpus.txt' which is used to build 'G.fst'
        """

        # Extract rules from the grammar file
        rules = self.parser.rules

        # Extract words and
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
        # print(words)

        # Create corpus.txt file and save the words list
        with open("corpus.txt", "w") as f:
            for word in words:
                f.write(word)


if __name__ == "__main__":
    import sys
    import pprint

    grammar_file = sys.argv[1]
    target = sys.argv[2]

    kaldi_gr = KaldiGrammar(grammar_file, target)
    tree = kaldi_gr.get_rule_element()
    pprint.pprint(tree)

    # # Get random sentence from the grammar
    # s = p.get_random_sentence('T')
    # print(s)
