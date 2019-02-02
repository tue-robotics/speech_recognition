#! /usr/bin/env python
#
# Python 2/3 compatibility
from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from builtins import *


class FsSymbolGenerator:
    """A class to generate the symbols file from FSA/FST text files to use with
    OpenFST"""
    def __init__(self, fname, fieldNumber):
        self.fname = fname
        self.fieldNumber = fieldNumber

    def generate(self):
        """Function to generate the symbols file"""
        words = set()

        with open(self.fname, 'r') as fsfile:
            for line in fsfile:
                fields = line.split(' ')
                if len(fields) > self.fieldNumber:
                    field = fields[self.fieldNumber].strip()
                    if (field):
                        words.add(field)

            # Always have the empty string as 0
            print("- 0")
            words.discard('-')
            for (ii, word) in enumerate(words, 1):
                print("%s %d" % (word, ii))


if __name__ == __main__:

    import sys
    if len(sys.argv) < 3 or "-h" in sys.argv or "--help" in sys.argv:
        print("""
        Usage: python makeSymbols file fieldNumber

        file: the textual FST/FSA file (.fst.txt or .fsa.txt usually), to extract the symbols from
        fieldNumber: which column of the file to take symbols from
            input symbols use fieldNumber of 2
            output symbols use fieldNumber of 3

        The Symbols Table is output to standard out, and can be piped into a file
        """)
        sys.exit(1)
    else:
        fname = sys.argv[1]
        fieldNumber = int(sys.argv[2])

    sym_gen = FsSymbolGenerator(fname, fieldNumber)
    sym_gen.generate()

