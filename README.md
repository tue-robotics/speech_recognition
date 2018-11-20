# LASeR: Linux Automation Speech Recognition [![Build Status](https://travis-ci.org/tue-robotics/speech_recognition.svg?branch=master)](https://travis-ci.org/tue-robotics/speech_recognition)
Speech recognition system based on [Kaldi-ASR](http://kaldi-asr.org/) toolkit. The backend library is written in C++ and has supports CUDA to build GPU accelerated models. Bash is used for most front-end operations and Python for data processing.

## Pre-requisites
1. Installation of [Kaldi-ASR_TUE](https://github.com/tue-robotics/kaldi)
2. Sourcing of ```setup.bash``` from [Kaldi-ASR_TUE](https://github.com/tue-robotics/kaldi) in ```.bashrc```

## Installation
1. Clone this repository
2. Source ```setup.bash``` of this repository in ```.bashrc```

## Testing
The installation can be tested by building a spoken digits recognition system:
```
./tests/digits.bash
```

## TODO
1. Complete data preparation tools for the stored datasets
