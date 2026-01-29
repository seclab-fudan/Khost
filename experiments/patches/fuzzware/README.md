## Usage:

#### Get Fuzzware

`git clone --recursive https://github.com/fuzzware-fuzzer/fuzzware.git && cd fuzzware && git checkout 28ce2dc3f888ec`

#### Apply the patches to add ARM64 support

`cp ROOT_DIR_Khost/experiments/patches/fuzzware/fuzzware.diff ./ && patch -p1 < fuzzware.diff`

#### Build      

To avoid environment conflicts, please install and run Fuzzware inside a Docker container. More details can be found in its [README.md](https://github.com/fuzzware-fuzzer/fuzzware/blob/main/README.md). 

`./build_docker.sh`

Then, you can create a container based on fuzzware image (fuzzware:latest), then, enter the conrainer and start fuzzing with AFL++ (--aflpp).

`fuzzware pipeline --skip-afl-cpufreq --aflpp DIR_TO_FIRMWARE`

