# LASeR: Linux Automation Speech Recognition
Speech recognition system based on [Kaldi-ASR](http://kaldi-asr.org/) toolkit. The backend library is written in C++ and has supports CUDA to build GPU accelerated models. Bash is used for most front-end operations and Python for data processing.

## Installation
The system can be setup by cloning the repository and executing:
```
sudo ./setup.bash --complete
```
Upon execution of the above command:
1. A local copy Kaldi is cloned into the root of the repository
2. Environment variables are added to ```.bashrc``` by creating an entry in the file ```.bash_exports```
3. Dependencies are installed
4. LASeR toolkit is built

The installation can be tested by building a spoken digits recognition system:
```
./tests/digits.bash
```

## TODO
1. Complete ```digits.bash```
2. Complete data preparation tools for the stored datasets
