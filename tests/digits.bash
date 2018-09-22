#! /usr/bin/env bash
#
# Test script for digits recognition
# 
# Arpit Aggarwal
# 2018 TechUnited @Home, TU Eindhoven

if [ [ -z "$KALDI_ROOT" ] || [ -z "$ASR_HOME" ] || [ -z "$ASR_LOG" ] ]
then
	echo -e "\e[35m\e[1m Environment variables do not exist \e[0m"
    exit 1
fi

echo -e "\e[33m\e[1m Testing Kaldi installation \e[0m"

cd $ASR_HOME/egs
tar -xvzf digits.tar.gz
cd digits
sed -i "1iexport KALDI_ROOT=$KALDI_ROOT" path.sh

mkdir steps utils
ln -s $KALDI/egs/wsj/s5/steps steps
ln -s $KALDI/egs/wsj/s5/utils utils


