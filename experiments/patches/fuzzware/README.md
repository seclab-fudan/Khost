## Usage:

#### Get Fuzzware

`git clone --recursive https://github.com/fuzzware-fuzzer/fuzzware.git && cd fuzzware && git checkout 28ce2dc3f888ec`

#### Apply the patches to add ARM64 support

`cp ROOT_DIR_Khost/experiments/patches/fuzzware/fuzzware.diff ./ && patch -p1 < fuzzware.diff`

#### Build Docker for Fuzzware

To avoid environment conflicts, please install and run Fuzzware inside a Docker container following the below commands.

**Create Image:** `./build_docker.sh`

**Create Container:**  `docker run --privileged=true -it -e LOCAL_USER_ID=id -u $USER -v "$PWD":/workspace/fuzzware --name fuzzware fuzzware:latest /bin/bash`

**Exit Container:**  `exit`

**Start Container:**  `docker start fuzzware`

#### Fuzzing Test Usage

**Copy Tests to Fuzzware's Workspace:** `cp -r   ROOT_DIR_Khost/experiments/patches/fuzzware/tests ./`

**Start Fuzzing with AFL++:**

`docker exec -it -u root fuzzware fuzzware pipeline --skip-afl-cpufreq --aflpp --base-inputs /workspace/fuzzware/tests/01_PLC/corpus /workspace/fuzzware/tests/01_PLC/`

#### Note:

More details can be found in its [README.md](https://github.com/fuzzware-fuzzer/fuzzware/blob/main/README.md).
