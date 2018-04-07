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

class Flags:
    """Class Flags takes no input arguments and initializes all flags to False"""
    def __init__(self):
        self.csv_check = False
        self.csv_header = False
        self.wav_path = False
        self.transcription = False
        self.gender = False
        self.utterance_id = False
        self.speaker_id = False

class Index:
    """Class Index has been defined to maintain the indices of the CSV file."""
    def __init__(self):
        self.wav_path = None
        self.transcription = None
        self.gender = None
        self.speaker_id = None
        self.utterance_id = None


class DataPreparation:
    """Class DataPreparation takes 2 input arguments

    Args:
        param1 (str): Absolute path of the CSV metadata file
        param2 (str): Absolute path of the root directory of Speech Recognition System
        param3 (str): Type of Dataset

    """
    def __init__(self, csv_path, srs_path, dataset):
        """ DataPreparation class constructor """

        file_extension = csv_path.split('.')[-1]
        if file_extension != 'csv':
            sys.exit("File type not '.csv'")

        if not os.path.exists(csv_path):
            sys.exit("Invalid CSV path")

        self.csv_path = csv_path
        self.csv_data_root = os.path.dirname(csv_path)

        self.flag = Flags()
        self.index = Index()

        self.csv_check()

        # Return the exact error
        if not self.flag.csv_check:
            sys.exit("CSV check failed")

        if srs_path[-1] == '/':
            srs_path = srs_path[:-1]

        if not os.path.exists(srs_path):
            sys.exit("Invalid path to Speech Recognition System root")

        # This section maybe buggy
        self.srs_path = srs_path
        self.srs_path_data = self.srs_path + "/data"
        self.srs_path_data_dataset = self.srs_path_data + "/" + dataset

        if os.path.exists(self.srs_path_data):
            data_dir_contents = os.listdir(self.srs_path_data)
            if data_dir_contents == []:
                os.mkdir(self.srs_path_data_dataset)
            else:
                data_dir_contents.remove('test')
                for directory in data_dir_contents:
                    shutil.rmtree(self.srs_path_data + "/" + directory)
                os.mkdir(self.srs_path_data_dataset)
        else:
            os.mkdir(self.srs_path_data)
            os.mkdir(self.srs_path_data_dataset)

        self.text()
        self.spk2gender()
        self.wavscp()
        self.spk2utt()

    def csv_check(self):
        """csvCheck performs the preliminary checks before the construction of
        the required data files.

        The following flow has been established:
            1. Read the CSV file to find the delimiter
            2. Read the Header of the CSV file
            3. Check whether the Header contains fields and store their indices:
                a. SPEAKER_ID
                b. WAV_PATH         (relative to the CSV file)
                c. transcription
                d. GENDER
                e. UTTERANCE_ID
            4. Read every row of the CSV file and check if all the wav paths exist
            5. Check whether number of wav files and transcriptions are equal
        """
        with open(self.csv_path, 'r') as csv_file:
            header = csv_file.readline()
            sniffer = csv.Sniffer()
            dialect = sniffer.sniff(header)
            self.csv_delimiter = dialect.delimiter

        with open(self.csv_path, 'r') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=self.csv_delimiter)
            csv_header = next(csv_reader)

            try:
                self.index.wav_path = csv_header.index("WAV_PATH")
                self.index.transcription = csv_header.index("TRANSCRIPTION")
                self.index.gender = csv_header.index("GENDER")
                self.index.speaker_id = csv_header.index("SPEAKER_ID")
                self.index.utterance_id = csv_header.index("UTTERANCE_ID")

            except:
                self.flag.csv_header = False
                self.flag.csv_check = False
                return
            else:
                self.flag.csv_header = True

            wav_files = 0
            transcriptions = 0
            gender_count = 0
            speaker_count = 0
            utterance_count = 0

            for row in csv_reader:
                wav_rel_path = row[self.index.wav_path]
                wav_path = self.csv_data_root + '/' + wav_rel_path
                if not os.path.exists(wav_path):
                    self.flag.wav_path = False
                    self.flag.csv_check = False
                    return
                wav_files += 1

                if row[self.index.transcription] != '':
                    transcriptions += 1

                if row[self.index.gender] != '':
                    gender_count += 1

                if row[self.index.speaker_id] != '':
                    speaker_count += 1

                if row[self.index.utterance_id] != '':
                    utterance_count += 1

            self.flag.wav_path = True

            self.total_files = wav_files

            if transcriptions != self.total_files:
                self.flag.transcription = False
                self.flag.csv_check = False
                return
            else:
                self.flag.transcription = True

            if gender_count != self.total_files:
                self.flag.gender = False
                self.flag.csv_check = False
                return
            else:
                self.flag.gender = True

            if speaker_count != self.total_files:
                self.flag.speaker_id = False
                self.flag.csv_check = False
                return
            else:
                self.flag.speaker_id = True

            if utterance_count != self.total_files:
                self.flag.utterance_id = False
                self.flag.csv_check = False
                return
            else:
                self.flag.utterance_id = True

            self.flag.csv_check = True

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
        with open(self.csv_path, 'r') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=self.csv_delimiter)
            row = next(csv_reader)

            iterations = self.total_files
            out = ''

            while iterations:
                row = next(csv_reader)

                sid = row[self.index.speaker_id]
                uid = row[self.index.utterance_id]
                transcription = row[self.index.transcription]

                fid = sid + 'U' + str(uid).zfill(5)

                out += fid + '\t' + transcription + '\n'
                iterations -= 1

        # The output file must not end with a newline
        out = out[:-1]
        with open(self.srs_path_data_dataset + '/text', 'w') as out_file:
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
        with open(self.csv_path, 'r') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=self.csv_delimiter)
            row = next(csv_reader)

            iterations = self.total_files
            out = ''

            # First speaker is not a new speaker
            row = next(csv_reader)
            sid = row[self.index.speaker_id]
            gender = row[self.index.gender]
            out += sid + '\t' + gender + '\n'
            iterations -= 1
            new_speaker = False

            while iterations:
                if new_speaker:

                    sid = row[self.index.speaker_id]
                    gender = row[self.index.gender]

                    out += sid + '\t' + gender + '\n'
                    iterations -= 1
                    new_speaker = False

                else:
                    row = next(csv_reader)

                    if row[self.index.speaker_id] == sid:
                        iterations -= 1
                        continue
                    else:
                        new_speaker = True

        # The output file must not end with a newline
        out = out[:-1]
        with open(self.srs_path_data_dataset + '/spk2gender', 'w') as out_file:
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
        with open(self.csv_path, 'r') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=self.csv_delimiter)
            row = next(csv_reader)

            iterations = self.total_files
            out = ''

            while iterations:
                row = next(csv_reader)

                sid = row[self.index.speaker_id]
                uid = row[self.index.utterance_id]
                wav_rel_path = row[self.index.wav_path]

                fid = sid + 'U' + str(uid).zfill(5)
                fpath = self.csv_data_root + '/' + wav_rel_path

                out += fid + '\t' + fpath + '\n'
                iterations -= 1

        # The output file must not end with a newline
        out = out[:-1]
        with open(self.srs_path_data_dataset + '/wav.scp', 'w') as out_file:
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
        with open(self.csv_path, 'r') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=self.csv_delimiter)
            row = next(csv_reader)

            iterations = self.total_files
            out = ''

            while iterations:
                row = next(csv_reader)

                sid = row[self.index.speaker_id]
                uid = row[self.index.utterance_id]

                fid = sid + 'U' + str(uid).zfill(5)

                out += fid + '\t' + sid + '\n'
                iterations -= 1

        # The output file must not end with a newline
        out = out[:-1]
        with open(self.srs_path_data_dataset + '/utt2spk', 'w') as out_file:
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
        if not os.path.exists(self.srs_path_data_dataset + "/utt2spk"):
            self.utt2spk()

        with open(self.srs_path_data_dataset + '/utt2spk', 'r') as utt2spk:
            iterations = self.total_files
            out = ''

            # First speaker is not a new speaker
            row = next(utt2spk).split()
            fid = row[0]
            sid = row[1]
            out += sid + ' ' + fid
            iterations -= 1
            new_speaker = False

            while iterations:
                if new_speaker:
                    out += '\n' + sid + ' ' + fid
                    new_speaker = False
                    iterations -= 1

                else:
                    row = next(utt2spk).split()
                    if row[1] == sid:
                        fid = row[0]
                        sid = row[1]
                        out += ' ' + fid
                        iterations -= 1

                    else:
                        new_speaker = True

        # The output file must not end with a newline
        # out = out[:-1]
        with open(self.srs_path_data_dataset + '/spk2utt', 'w') as out_file:
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
        dataset = sys.argv[3]

        DataPreparation(csv_path, srs_path, dataset)

 # Add the main() subroutine
if __name__ == '__main__':
    main()
