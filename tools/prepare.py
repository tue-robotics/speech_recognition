#!/usr/bin/env python3
"""This module has been written to prepare the files and directory structure
required by Kaldi-ASR to develop Acoustic Models."""

import os

def text(root_dir):
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
