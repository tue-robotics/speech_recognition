# LASeR: Linux Automation Speech Recognition [![Build Status](https://travis-ci.org/tue-robotics/speech_recognition.svg?branch=master)](https://travis-ci.org/tue-robotics/speech_recognition)
Speech recognition system based on [Kaldi-ASR](http://kaldi-asr.org/) toolkit. The backend library is written in C++ and supports CUDA to build GPU accelerated models. Python is used for most front-end operations and data processing, complementing it with Bash time to time.

## Pre-requisites
1. Installation of CUDA
2. Installation of [Kaldi-ASR_TUE](https://github.com/tue-robotics/kaldi)
3. Sourcing of ```setup.bash``` from [Kaldi-ASR_TUE](https://github.com/tue-robotics/kaldi) in ```.bashrc```

## Installation
### Using `tue-env`
```
tue-get install ros-speech_recognition
```
### From source
1. Clone this repository
2. Unzip the latest of the released models into `~/data/speech_models/model/`
3. Install `package.xml` dependencies
4. Add repository to `catkin` workspace

## Testing
### Model building
The installation can be tested by building a spoken digits recognition system:
```
./tests/digits.bash
```

Upon completion, the last 4 lines displayed must be
```
%WER 12.00 [ 12 / 100, 0 ins, 0 del, 12 sub ] exp/mono/decode/wer_10
%WER 6.00 [ 6 / 100, 0 ins, 0 del, 6 sub ] exp/tri1/decode/wer_10

==== Execution completed ====
```

### ROS Package test
To test, run the following two commands:
```
roslaunch speech_recognition kaldi_speech_recognition.launch
```
```
rosrun hmi test_query -s hmi/kaldi_speech_recognition --grammar "T -> yes | no" T
```

## TODO
1. Complete data preparation tools for the stored datasets
