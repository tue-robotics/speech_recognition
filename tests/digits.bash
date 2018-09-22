#! /usr/bin/env bash
#
# Test script for digits recognition
# 
# Arpit Aggarwal
# 2018 TechUnited @Home, TU Eindhoven

kaldi_test()
{
	log "Testing Kaldi installation"
	cd $ASR_HOME/egs
	tar -xvzf digits.tar.gz
	cd digits
	sed -i "1iexport KALDI_ROOT=$KALDI" path.sh
	mkdir steps utils
	ln -s $KALDI/egs/wsj/s5/steps steps
	ln -s $KALDI/egs/wsj/s5/utils utils

}

