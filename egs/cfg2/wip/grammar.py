#! /usr/bin/env python
import sys
from grammar_parser import cfgparser

p = cfgparser.CFGParser.fromfile('grammar.fcfg')
s = p.get_random_sentence('T')
print(s)
