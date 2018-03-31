#!/usr/bin/env python3
"""This module has been written to prepare the files and directory structure
required by Kaldi-ASR to develop Acoustic Models."""

import os
import sys
import csv

class DataPreparation:
    """Class DataPreparation takes 2 input arguments

    Args:
        param1 (str): Absolute path of the CSV metadata file
        param2 (str): Absolute path of the root directory of Speech Recognition System

    """
    def __init__(self, csv_path, srs_path):
        """ DataPreparation class constructor """
        if !os.path.exists(csv_path):
            sys.exit("Invalid CSV path")

        self.CSV_PATH = csv_path
        self.DATA_ROOT = os.path.dirname(csv_path)

        if srs_path[-1] == '/':
            srs_path = srs_path[:-1]

        if !os.path.exists(srs_path):
            sys.exit("Invalid path to Speech Recognition System root")

        self.SRS_PATH = srs_path

    def csvCheck(self):
        """csvCheck performs the preliminary checks before the construction of
        the required data files.

        The following flow has been established:
            1. Read the CSV file to find the delimiter
            2. Read the Header of the CSV file
            3. Check whether the Header contains fields and store their indices:
                a. SPEAKER_ID
                b. WAV_PATH         (relative to the CSV file)
                c. TRANSCRIPTION
                d. GENDER
            4. Read every row of the CSV file and check if all the wav paths exist
            5. Check whether number of wav files and transcriptions are equal
        """
        with open(self.CSV_PATH, 'r') as csv_file:
            header = csv_file.readline()
            sniffer = csv.Sniffer()
            dialect = sniffer.sniff(header)
            self.CSV_DELIMITER = dialect.delimiter

        with open(self.CSV_PATH, 'r') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter = self.CSV_DELIMITER)
            self.CSV_HEADER = next(csv_reader)

            try:
                wav_path_index = self.CSV_HEADER.index("WAV_PATH")
                transcription_index = self.CSV_HEADER.index("TRANSCRIPTION")
            except:
                self.FLAG.CSV_HEADER = False
                return
            else:
                self.FLAG.CSV_HEADER = True

            wav_files = 0
            transcriptions = 0
            for row in csv_reader:
                wav_rel_path = row[wav_path_index]
                wav_path = self.DATA_ROOT + '/' + wav_rel_path
                if !os.path.exists(wav_path):
                    self.FLAG.WAV_PATH = False
                    return

                wav_files += 1
                if row[transcription_index] != '':
                    transcriptions += 1

            self.FLAG.WAV_PATH = True
            self.WAV_FILES = wav_files
            self.TRANSCRIPTIONS = transcriptions

            if self.TRANSCRIPTIONS != self.WAV_FILES:
                self.FLAG.CSV_CHECK = False

            self.FLAG.CSV_CHECK = True

    def text(self):
        """Prepares the file 'text' """
        if root_dir[-1] != '/':
            root_dir += '/'

        text_dir = root_dir + 'txt/'
        text_contents = os.listdir(text_dir)
        text_contents.sort()
        out = ''
        for text_file in text_contents:
            out += text_file.split('.')[0] + '\t'
            fname = open(text_dir + text_file, 'r')
            fname_full = fname.read()
            out += fname_full + '\n'
            fname.close()
        out = out[:-1]
        tfname = open(root_dir + 'text', 'w')
        tfname.write(out)
        tfname.close()



def spk2gender(root_dir):
    """Prepares the file 'spk2gender'"""

def wavscp(root_dir):
    """Prepares the file 'wav.scp' """
    if root_dir[-1] != '/':
        root_dir += '/'
    wav_dir = root_dir + 'wav/'
    wav_contents = os.listdir(wav_dir)
    wav_contents.sort()
    out = ''
    for wav_file in wav_contents:
        out += wav_file.split('.')[0] + '\t' + wav_dir + wav_file + '\n'
    out = out[:-1]
    wfname = open(root_dir + 'wav.scp', 'w')
    wfname.write(out)
    wfname.close()

def utt2spk(root_dir):
    """Prepares the file 'utt2spk' """
    if root_dir[-1] != '/':
        root_dir += '/'
    text_dir = root_dir + 'txt/'
    text_contents = os.listdir(text_dir)
    text_contents.sort()
    out = ''
    for text_file in text_contents:
        fname = text_file.split('.')[0]
        out += fname + '\t' + fname[:-4] + '\n'
    out = out[:-1]
    uttfname = open(root_dir + 'utt2spk', 'w')
    uttfname.write(out)
    uttfname.close()

def spk2utt(root_dir):
    """Prepares the file 'spk2utt' """
    if root_dir[-1] != '/':
        root_dir += '/'
    text_dir = root_dir + 'txt/'
    text_contents = os.listdir(text_dir)
    text_contents.sort()
    spk_begin = (text_contents[0].split('.'))[0][:-4]
    out = spk_begin
    for text_file in text_contents:
        fname = text_file.split('.')[0]
        if fname[:-4] == spk_begin:
            out += ' ' + fname
        else:
            spk_begin = fname[:-4]
            out += '\n' + spk_begin + ' ' + fname
    spkfname = open(root_dir + 'spk2utt', 'w')
    spkfname.write(out)
    spkfname.close()
