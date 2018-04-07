#!/usr/bin/env python3
"""This module has been written to prepare the files and directory structure
required by Kaldi-ASR to develop Acoustic Models.

Structure of the CSV file:

-----------------------------------------------------------------
| SPEAKER_ID | UTTERANCE_ID | WAV_PATH | TRANSCRIPTION | GENDER |
-----------------------------------------------------------------

TODO:
1. Check correctness of file endings for each created data file
2. Analyse the best suited format for value of fields SPEAKER_ID and UTTERANCE_ID.
   Change the following methods accordingly:
        a. text
        b. wav.scp
        c. spk2utt
        d. utt2spk
3. Analyse the best suited naming structure for each FILE
"""

import os
import sys
import csv
import shutil

class DataPreparation:
    """Class DataPreparation takes 2 input arguments

    Args:
        param1 (str): Absolute path of the CSV metadata file
        param2 (str): Absolute path of the root directory of Speech Recognition System
        param3 (str): Type of Dataset

    """
    def __init__(self, csv_path, srs_path, dataSet):
        """ DataPreparation class constructor """

        file_extension = csv_path.split('.')[-1]
        if file_extension != 'csv':
            sys.exit("File type not '.csv'")

        if !os.path.exists(csv_path):
            sys.exit("Invalid CSV path")

        self.CSV_PATH = csv_path
        self.CSV_DATA_ROOT = os.path.dirname(csv_path)

        self.csvCheck

        # Return the exact error
        if !self.FLAG.CSV_CHECK:
            sys.exit("CSV check failed")

        if srs_path[-1] == '/':
            srs_path = srs_path[:-1]

        if !os.path.exists(srs_path):
            sys.exit("Invalid path to Speech Recognition System root")

        # This section maybe buggy
        self.SRS_PATH = srs_path
        self.SRS_PATH_DATA = self.SRS_PATH + "/data"
        self.SRS_PATH_DATA_DATASET = self.SRS_PATH_DATA + "/" + dataSet

        if os.path.exists(self.SRS_PATH_DATA):
            data_dir_contents = os.listdir(self.SRS_PATH_DATA)
            if data_dir_contents == []:
                os.mkdir(self.SRS_PATH_DATA_DATASET)
            else:
                data_dir_contents.remove('test')
                for directory in data_dir_contents:
                    shutil.rmtree(self.SRS_PATH_DATA + "/" + directory)
                os.mkdir(self.SRS_PATH_DATA_DATASET)
        else:
            os.mkdir(self.SRS_PATH_DATA)
            os.mkdir(self.SRS_PATH_DATA_DATASET)

        self.text
        self.spk2gender
        self.wavscp
        self.spk2utt

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
                e. UTTERANCE_ID
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
                self.INDEX.WAV_PATH = self.CSV_HEADER.index("WAV_PATH")
                self.INDEX.TRANSCRIPTION = self.CSV_HEADER.index("TRANSCRIPTION")
                self.INDEX.GENDER = self.CSV_HEADER.index("GENDER")
                self.INDEX.SPEAKER_ID = self.CSV_HEADER.index("SPEAKER_ID")
                self.INDEX.UTTERANCE_ID = self.CSV_HEADER.index("UTTERANCE_ID")

            except:
                self.FLAG.CSV_HEADER = False
                self.FLAG.CSV_CHECK = False
                return
            else:
                self.FLAG.CSV_HEADER = True

            wav_files = 0
            transcriptions = 0
            gender_count = 0
            speaker_count = 0
            utterance_count = 0

            for row in csv_reader:
                wav_rel_path = row[self.INDEX.WAV_PATH]
                wav_path = self.CSV_DATA_ROOT + '/' + wav_rel_path
                if !os.path.exists(wav_path):
                    self.FLAG.WAV_PATH = False
                    self.FLAG.CSV_CHECK = False
                    return
                wav_files += 1

                if row[self.INDEX.TRANSCRIPTION] != '':
                    transcriptions += 1

                if row[self.INDEX.GENDER] != '':
                    gender_count += 1

                if row[self.INDEX.SPEAKER_ID] != '':
                    speaker_count += 1

                if row[self.INDEX.UTTERANCE_ID] != '':
                    utterance_count += 1

            self.FLAG.WAV_PATH = True

            self.WAV_FILES = wav_files
            self.TRANSCRIPTIONS = transcriptions
            self.GENDER_COUNT = gender_count
            self.SPEAKER_COUNT = speaker_count
            self.UTTERANCE_COUNT = utterance_count

            if self.TRANSCRIPTIONS != self.WAV_FILES:
                self.FLAG.TRANSCRIPTION = False
                self.FLAG.CSV_CHECK = False
                return
            else:
                self.FLAG.TRANSCRIPTION = True

            if self.GENDER_COUNT != self.WAV_FILES:
                self.FLAG.GENDER = False
                self.FLAG.CSV_CHECK = False
                return
            else:
                self.FLAG.GENDER = True

            if self.SPEAKER_COUNT != self.WAV_FILES:
                self.FLAG.SPEAKER_ID = False
                self.FLAG.CSV_CHECK = False
                return
            else:
                self.FLAG.SPEAKER_ID = True

            if self.UTTERANCE_COUNT != self.WAV_FILES:
                self.FLAG.UTTERANCE_ID = False
                self.FLAG.CSV_CHECK = False
                return
            else:
                self.FLAG.UTTERANCE_ID = True

            self.FLAG.CSV_CHECK = True

    def text(self):
        """text prepares the file 'text' in the DATASET directory.

        The following flow has been established:
        1. Read the CSV file
        2. From each row extract:
            a. SPEAKER_ID
            b. UTTERANCE_ID
            c. TRANSCRIPTION
        3. Make file id "<SPEAKER_ID>U<UTTERANCE_ID>"
        4. Write an output file where each line has the structure:
            <FILE_ID><Tab_space><TRANSCRIPTION>
        """
        with open(self.CSV_PATH, 'r') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter = self.CSV_DELIMITER)
            row = next(csv_reader)

            iterations = self.WAV_FILES
            out = ''

            while iterations:
                row = next(csv_reader)

                sid = row[self.INDEX.SPEAKER_ID]
                uid = row[self.INDEX.UTTERANCE_ID]
                transcription = row[self.INDEX.TRANSCRIPTION]

                fid = sid + 'U' + str(uid).zfill(5)

                out += fid + '\t' + transcription + '\n'
                iterations -= 1

        # The output file must not end with a newline
        out = out[:-1]
        with open(self.SRS_PATH_DATA_DATASET + '/text', 'w') as out_file:
            out_file.write(out)

    def spk2gender(self):
        """spk2gender prepares the file 'spk2gender' in the DATASET directory.

        The following flow has been established:
        1. Read the CSV file
        2. Read the first row (First Speaker) and extract the gender details
        3. Search for a new speaker and extract its gender details
        4. Write an output file where each line has the structure:
            <SPEAKER_ID><Tab_space><GENDER>
        """
        with open(self.CSV_PATH, 'r') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter = self.CSV_DELIMITER)
            row = next(csv_reader)

            iterations = self.WAV_FILES
            out = ''

            # First speaker is not a new speaker
            row = next(csv_reader)
            sid = row[self.INDEX.SPEAKER_ID]
            gender = row[self.INDEX.GENDER]
            out += sid + '\t' + gender + '\n'
            iterations -= 1
            newSpeaker = False

            while iterations:
                if newSpeaker:

                    sid = row[self.INDEX.SPEAKER_ID]
                    gender = row[self.INDEX.GENDER]

                    out += sid + '\t' + gender + '\n'
                    iterations -= 1
                    newSpeaker = False

                else:
                    row = next(csv_reader)

                    if row[self.INDEX.SPEAKER_ID] == sid:
                        iterations -= 1
                        continue
                    else:
                        newSpeaker = True

        # The output file must not end with a newline
        out = out[:-1]
        with open(self.SRS_PATH_DATA_DATASET + '/spk2gender', 'w') as out_file:
            out_file.write(out)

    def wavscp(self):
        """wavscp prepares the file 'wav.scp' in the DATASET directory.

        The following flow has been established:
        1. Read the CSV file
        2. From each row extract:
            a. SPEAKER_ID
            b. UTTERANCE_ID
            c. WAV_PATH     (relative to the CSV file)
        3. Make FILE_ID "<SPEAKER_ID>U<UTTERANCE_ID>"
        4. Make FILE_PATH "<CSV_DATA_ROOT>/<WAV_PATH>"
        5. Write an output file where each line has the structure:
            <FILE_ID><Tab_space><FILE_PATH>
        """
        with open(self.CSV_PATH, 'r') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter = self.CSV_DELIMITER)
            row = next(csv_reader)

            iterations = self.WAV_FILES
            out = ''

            while iterations:
                row = next(csv_reader)

                sid = row[self.INDEX.SPEAKER_ID]
                uid = row[self.INDEX.UTTERANCE_ID]
                wav_rel_path = row[self.INDEX.WAV_PATH]

                fid = sid + 'U' + str(uid).zfill(5)
                fpath = self.CSV_DATA_ROOT + '/' + wav_rel_path

                out += fid + '\t' + fpath + '\n'
                iterations -= 1

        # The output file must not end with a newline
        out = out[:-1]
        with open(self.SRS_PATH_DATA_DATASET + '/wav.scp', 'w') as out_file:
            out_file.write(out)

    def utt2spk(self):
        """utt2spk prepares the file 'utt2spk' in the DATASET directory.

        The following flow has been established:
        1. Read the CSV file
        2. From each row extract:
            a. SPEAKER_ID
            b. UTTERANCE_ID
        3. Make FILE_ID "<SPEAKER_ID>U<UTTERANCE_ID>"
        4. Write an output file where each line has the structure:
            <FILE_ID><Tab_space><SPEAKER_ID>
        """
        with open(self.CSV_PATH, 'r') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter = self.CSV_DELIMITER)
            row = next(csv_reader)

            iterations = self.WAV_FILES
            out = ''

            while iterations:
                row = next(csv_reader)

                sid = row[self.INDEX.SPEAKER_ID]
                uid = row[self.INDEX.UTTERANCE_ID]

                fid = sid + 'U' + str(uid).zfill(5)

                out += fid + '\t' + sid + '\n'
                iterations -= 1

        # The output file must not end with a newline
        out = out[:-1]
        with open(self.SRS_PATH_DATA_DATASET + '/utt2spk', 'w') as out_file:
            out_file.write(out)

    def spk2utt(self):
        """spk2utt prepares the file 'spk2utt' in the DATASET directory.

        The following flow has been established:
        1. Read 'utt2spk' from the DATASET directory (if missing, create it)
        2. From each row extract:
           a. FILE_ID
           b. SPEAKER_ID
        3. Write an output file where each line has the structure:
           <SPEAKER_ID> <FILE_ID_1> <FILE_ID_2> ... <FILE_ID_END>
        """
        if not os.path.exists(self.SRS_PATH_DATA_DATASET + "/utt2spk"):
            self.utt2spk

        with open(self.SRS_PATH_DATA_DATASET + '/utt2spk', 'r') as utt2spk:
            iterations = self.WAV_FILES
            out = ''

            # First speaker is not a new speaker
            row = next(utt2spk).split()
            fid = row[0]
            sid = row[1]
            out += sid + ' ' + fid
            iterations -= 1
            newSpeaker = False

            while iterations:
                if newSpeaker:
                    out += '\n' + sid + ' ' + fid
                    newSpeaker = False
                    iterations -= 1

                else:
                    row = next(utt2spk).split()
                    if row[1] == sid:
                        fid = row[0]
                        sid = row[1]
                        out += ' ' + fid
                        iterations -= 1

                    else:
                        newSpeaker = True

        # The output file must not end with a newline
        # out = out[:-1]
        with open(self.SRS_PATH_DATA_DATASET + '/spk2utt', 'w') as out_file:
            out_file.write(out)

def main():
    """When script is executed as __main__:

    Args:
        param1 (str): Absolute path of the CSV metadata file
        param2 (str): Absolute path of the root directory of Speech Recognition System
        param3 (str): Type of Dataset
    """
    if not len(sys.argv) == 4:
        sys.exit("Not enough input arguments")

    else:
        csv_path = sys.argv[1]
        srs_path = sys.argv[2]
        dataSet = sys.argv[3]

        DataPreparation(csv_path, srs_path, dataSet)

 # Add the main() subroutine
if __name__ == '__main__':
    main()
