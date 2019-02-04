#! /usr/bin/env python
#
# Python 2/3 compatibility
from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from builtins import *

import os.path
import sys

class FstCompiler:
    """A class to generate fst binaries from raw text files or strings"""
    def __init__(self, fpath, outpath):
        if not os.path.exists(fpath):
            self.error("File path '{}' does not exist".format(fpath))



        fname_split = os.path.basename(fpath).split('.')
        try:
            fname = fname_split[0]
            fstype = fname_split[1]
            fext = fname_split[2]

            if not fext == ".txt":
                self.error("Incorrect extension '{}' of '{}'. Need '.txt' file".format(fext, fpath))

            if not (fstype == "fsa" or fstype == "fst"):
                self.error("Unknown fst type '{}' in '{}'. Should either be 'fst' or 'fsa'".format(fstype, fpath))

        except:
            self.error("Error! Incorrect input file '{}'. Check usage to understand the correct file input".format(fpath))

        self.fpath = fpath
        self.fname = fname
        self.fstype = fstype

        self.fsFile = self.fname + self.fstype
        self.isymsFile = self.fname + "isyms"
        self.osymsFile = self.fname + "osyms"

    def generateSymsFiles(self):
        """Method to generate the symbols files isyms and osyms"""
        words = set()

        self.isymsFileHandle = open(self.isymsFile, 'w')
        self.isymsFileHandle.write("- 0")

        # FSAs have only one field, hence 2 by default is added
        self.fieldFileDict = {2: self.isymsFileHandle}

        # FSTs have two fields
        if self.fstype == "fst":
            self.osymsFileHandle = open(self.osymsFile, 'w')
            self.osymsFileHandle.write("- 0")
            self.fieldFileDict[3] = self.osymsFileHandle

        # Read the raw text file
        with open(self.fpath, 'r') as fsfiletxt:
            lines = fsfiletxt.readlines()
            lines = [line.strip().split(' ') for line in lines]

        for index in self.fieldFileDict:
            fh = self.fieldFileDict[index]
            field_count = 1
            for line in lines:
                try:
                    field = line[index]
                except:
                    pass
                else:
                    fh.write("{} {}".format(field_count + 1, field))
            fh.close()

    def compile(self):
        """Method to compile FSA/FST after generating symbols files using
        generateSymsFiles"""


    def error(self, *args, **kwargs):
        """Error function of FstCompiler class"""
        print("[Kaldi-Grammar_Parser]", *args, file=sys.stderr, **kwargs)
        sys.exit(1)

    def warning(self, *args, **kwargs):
        """Warning method of FstCompiler class"""
        print("[Kaldi-Grammar_Parser]", *args, file=sys.stdout, **kwargs)



if __name__ == "__main__":

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

