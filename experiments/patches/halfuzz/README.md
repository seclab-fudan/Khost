## Usage：

#### Get hal-fuzz (Halucinator-based fuzzer)

`git clone https://github.com/ucsb-seclab/hal-fuzz.git --recursive && cd hal-fuzz && git checkout 1446408e64b25aa`

#### Apply the patches to add ARM64 support

`cp ROOT_DIR_Khost/experiments/patches/hal-fuzz/halfuzz.diff ./ && patch -p1 <halfuzz.diff`

#### Build Docker for HALucinator

To avoid environment conflicts, please install and run HALucinator inside a Docker container following the below commands.

**Create Image:** `docker build -t halfuzz:latest .`

**Create Container:**  `docker run --privileged=true -it -e LOCAL_USER_ID=id -u $USER -v "$PWD":/workspace/halfuzz --name halfuzz halfuzz:latest /bin/bash`

**Exit Container:**  `exit`

**Start Container:**  `docker start halfuzz`

#### Fuzzing Test Usage

**Copy Tests to Fuzzware's Workspace:** `cp -r   ROOT_DIR_Khost/experiments/patches/halfuzz/fuzz++ ./`

**Start Fuzzing with AFL++:**

`docker exec -it -uroot halfuzz /workspace/halfuzz/fuzz++/01_PLC/fuzz_start.sh`

#### Note:

More details can be found in its [README.md](https://github.com/ucsb-seclab/hal-fuzz/blob/master/README.md) .


