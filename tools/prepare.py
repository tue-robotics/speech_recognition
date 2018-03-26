#!/usr/bin/env python3

import os
import shutil
import sys
import time

def text(root_dir):
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
    if root_dir[-1] != '/':
        root_dir += '/'
    wav_dir = root_dir + 'wav/'
    wav_contents = os.listdir(wav_dir)
    wav_contents.sort()
    out = ''
    for f in wav_contents:
        out += f.split('.')[0] + '\t' + wav_dir + f + '\n'
    out = out[:-1]
    wfname = open(root_dir + 'wav.scp', 'w')
    wfname.write(out)
    wfname.close()

def utt2spk(root_dir):
    if root_dir[-1] != '/':
        root_dir += '/'
    text_dir = root_dir + 'txt/'
    text_contents = os.listdir(text_dir)
    text_contents.sort()
    out = ''
    for f in text_contents:
        fname = f.split('.')[0]
        out += fname + '\t' + fname[:-4] + '\n'
    out = out[:-1]
    uttfname = open(root_dir + 'utt2spk', 'w')
    uttfname.write(out)
    uttfname.close()

def spk2utt(root_dir):
    if root_dir[-1] != '/':
        root_dir += '/'
    text_dir = root_dir + 'txt/'
    text_contents = os.listdir(text_dir)
    text_contents.sort()
    spk_begin = (text_contents[0].split('.'))[0][:-4]
    out = spk_begin
    for f in text_contents:
        fname = f.split('.')[0]
        if fname[:-4] == spk_begin:
            out += ' ' + fname
        else:
            spk_begin = fname[:-4]
            out += '\n' + spk_begin + ' ' + fname
    spkfname = open(root_dir + 'spk2utt', 'w')
    spkfname.write(out)
    spkfname.close()


path = input("Enter path to the transcribed data: ")
if not os.path.exists(path):
    print("Transcribed data directory path does not exist")
    sys.exit(-1)

if path[-1] != '/':
    path += '/'

out = input("Enter path to output directory: ")
if not os.path.exists(out):
    print("Output data directory path does not exist")
    sys.exit(-1)

if out[-1] != '/':
    out += '/'

date = time.strftime("%Y-%m-%d-%H:%M")
out = out + "KalDAT_" + date + '/'
os.mkdir(out)

DnM = out + 'DnM/'
os.mkdir(DnM)
DnM_text = DnM + 'txt/'
DnM_wav = DnM + 'wav/'
os.mkdir(DnM_text)
os.mkdir(DnM_wav)

Comm = out + 'Comm/'
os.mkdir(Comm)
Comm_text = Comm +'txt/'
Comm_wav = Comm + 'wav/'
os.mkdir(Comm_text)
os.mkdir(Comm_wav)

path_contents = os.listdir(path)

for spk in path_contents:
    spk_path = path + spk + '/'
    spk_contents = os.listdir(spk_path)

    if (spk[8] == 'A') or (spk[8] == 'F'):
        for f in spk_contents:
            if f.split('.')[-1] == 'wav':
                shutil.copy(spk_path + f, DnM_wav)
            if f.split('.')[-1] == 'txt':
                shutil.copy(spk_path + f, DnM_text)

    if (spk[8] == 'B') or (spk[8] == 'C') or (spk[8] == 'E'):
        for f in spk_contents:
            if f.split('.')[-1] == 'wav':
                shutil.copy(spk_path + f, Comm_wav)
            if f.split('.')[-1] == 'txt':
                shutil.copy(spk_path + f, Comm_text)

text(DnM)
text(Comm)
wavscp(DnM)
wavscp(Comm)
utt2spk(DnM)
utt2spk(Comm)
spk2utt(DnM)
spk2utt(Comm)
