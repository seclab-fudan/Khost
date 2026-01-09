## Usage：

#### Get hal-fuzz (Halucinator-based fuzzer)

`git clone https://github.com/ucsb-seclab/hal-fuzz.git --recursive && cd hal-fuzz && git checkout 1446408e64b25aa`

#### Apply the patches to add ARM64 support

`cp ROOT_DIR_Khost/experiments/patches/hal-fuzz/halfuzz.diff ./unicorn-mode/patches && cp ROOT_DIR_Khost/experiments/patches/hal-fuzz/build_unicorn_support.sh  unicorn-mode`

#### Update AFL++  Version (v4.3)

`git clone https://github.com/AFLplusplus/AFLplusplus && git -C AFLplusplus checkout b89727bea903aec80`

#### Build       

Build hal-fuzz according to the instructions in its [README.md](https://github.com/ucsb-seclab/hal-fuzz/blob/master/README.md). To avoid environment conflicts, we recommend installing and running hal-fuzz inside a Docker container.

#### Use AFL++ Backe

To enable afl++ backend, please to replace the `FUZZ=$WORKDIR/afl-fuzz` as `FUZZ=$WORKDIR_AFLplusplus/afl-fuzz`


