## Usage:

#### Get Fuzzware

`git clone --recursive https://github.com/fuzzware-fuzzer/fuzzware.git && cd fuzzware && git checkout 28ce2dc3f888ec`

#### Apply the patches to add ARM64 support

`cp ROOT_DIR_Khost/experiments/patches/fuzzware/fuzzware.diff ./ && cd emulator && patch -p1 < ../fuzzware.diff && cd ../`

#### Update AFL++  Version (v4.3)

`cp ROOT_DIR_Khost/experiments/patches/fuzzware/get_afl.sh emulator`

#### Build       

Build Fuzzware according to the instructions in its [README.md](https://github.com/fuzzware-fuzzer/fuzzware/blob/main/README.md). To avoid environment conflicts, please install and run Fuzzware inside a Docker container. 

#### About AFL++ Mode

For details on how to use the AFL++ mode, please refer to the corresponding [issue](https://github.com/fuzzware-fuzzer/fuzzware/issues/29).
