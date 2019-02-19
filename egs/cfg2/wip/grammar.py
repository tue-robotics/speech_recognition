#! /usr/bin/env python
import sys
from grammar_parser.cfgparser import CFGParser

words = set()

p = CFGParser.fromfile('current_grammar.fcfg')
rules = p.rules

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

with open("corpus.txt", "w") as f:
    for word in words:
        f.write(word)



print(words)


#s = p.get_random_sentence('T')
#print(s)
