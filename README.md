# **Khost**: KVM-based Near Native MCU Firmware Rehosting

## Overview

```
.
├── README.md       # usage documents
├── build.sh        # script for compiling the project (AFL++-backend)
├── clean.sh        # script for cleaning the project
├── src             # source code
│   ├── common      # common files
│   ├── runner      # rehost module
│   ├── engine      # key engine
│   ├── fuzzer      # afl++ fuzzer backend
├── experiments     # experiment configuration
│   ├── fuzz        # configuration files for afl++
│   ├── libafl_fuzz # configuration files for libafl
│   ├── performance # configuration files for coremark and simbench
│   ├── patches     # patches used to support our tests
├── scripts         # a series of helper scripts
├── LibAFL          # source code of libafl backend
├── build.rs        # script for compiling the project (LibAFL++-backend)
├── rust-toolchain  # rustup configuration
├── Cargo.toml      # Cargo configuration
└── Cargo.lock      # Cargo lock
```

## Environment Requirement

Khost is developed and evaluated on **Ubuntu 22.04**, running on either a **Raspberry Pi 4B (8GB RAM and 64GB Flash)** or a **Huawei KunPeng Server (TaiShan 2280)**. To reproduce our results, we recommend using the same hardware and system configuration.

> Note: On the Raspberry Pi 4B, environment setup and the subsequent compilation take approximately **2 hours**.

### Memory and Swap Requirements

Khost is memory-intensive, so **8 GB** of RAM is recommended. If your hardware has less memory, you must configure **swap space** to prevent the program from being terminated by the Linux **Out-Of-Memory (OOM) killer**. You can refer to the following commands to complete this.

```
sudo swapoff /swapfile
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

### KVM Permissions

Access to `/dev/kvm` is restricted to users in the kvm group. If you encounter the error `Permission denied accessing /dev/kvm`, add your user to the kvm group using `sudo usermod -aG kvm $USER`. To take effect, you must **fully log out** and log back in after running this command. You can verify the change using `groups | grep kvm`. **Alternatively**, if the system is accessed by logging in as the root user, the above KVM permission configuration steps are not required.

If `/dev/kvm` is still inaccessible after re-login, ensure that your kernel has KVM support enabled.

### Patch Kernel Bug

Due to a bug in the Linux kernel that prevents vCPU restoration in AArch32 system mode, the kernel must be updated to version 5.15.0.1070 or later to incorporate the corresponding fix. More details can be found in the following upstream KVM patch: https://patchwork.kernel.org/project/kvm/patch/20240524141956.1450304-3-maz@kernel.org/

On Raspberry Pi 4B, you can either run `apt install linux-image-raspi=5.15.0.1092.90 && reboot`, or download the kernel from [linux-kernel-for raspberry](https://launchpad.net/ubuntu/+source/linux-raspi/5.15.0-1070.73) and update manually.

### Package Dependencies

Please install the packages that Khost requires according to the following terminal.

`apt install curl vim cmake make bc git gcc g++ llvm-dev gcc-arm-none-eabi libyaml-cpp-dev pip tmux ninja-build meson libglib2.0-dev python3-virtualenv python3-virtualenvwrapper `

Keystone: `git clone https://github.com/keystone-engine/keystone.git && cd keystone && git checkout fb92f32391c6cced8 && mkdir build && cd build && ../make-share.sh && make install && ldconfig && cd ../../`

Capstone: `git clone https://github.com/capstone-engine/capstone.git && cd capstone && git checkout 106f7d3b949f5dd8e6c && ./cmake.sh && make -j && make install && ldconfig && cd ../`

<a id="modeling-module"></a>
Modeling Module: `pip3 install -r experiments/patches/fuzzware/requirements-modeling.txt && git clone --recursive https://github.com/fuzzware-fuzzer/fuzzware.git && cd fuzzware && git checkout 28ce2dc3f888ec && cd modeling && python3 setup.py install && cd ../../`

## Compile and Verify

Go to the root directory of Khost and run `./clean.sh && ./build.sh` to compile Khost.

After the compilation completes, you can verify the build by running `./build/runner/khost -h` and `./build/fuzzer/khost-afl-fuzz -h`.  Both commands will display their respective help messages.

## Performance Test

#### Configure and Build QEMU

You can configure and build QEMU using the following commands (all operations are performed in the root directory of Khost).

Download & decompress: `wget https://download.qemu.org/qemu-8.2.10.tar.bz2 && tar xvf qemu-8.2.10.tar.bz2`

Adjust RAM/FLASH: `cp experiments/patches/qemu/stm32f405_soc.h qemu-8.2.10/include/hw/arm/`

Configure & build: `cd qemu-8.2.10 && mkdir build && cd build && ../configure --target-list="arm-softmmu" --disable-capstone && make -j$(($(nproc) - 1)) && cd ../../`

####  On CoreMark-PRO

##### ##QEMU-based Reshoting

Command: `./scripts/coremark_test_qemu.sh`

Example output (The **Time Interval** line presents the result, and the following three lines are used as references):

```
################## Round 0 ###################
Ananlyzing experiments/performance/coremark-pro-mcu/cjpeg/coremark-pro.elf
Plugin watching ADDR1 = 0x8000e7a, ADDR2 = 0x8000e8a
[Start ] First exec at 0x8000e7a | Time: 1766385191150 ms
[Finish] First exec at 0x8000e8a | Time: 1766385222059 ms
[Time Interval]: 30909 ms

real    0m31.030s
user    0m39.119s
sys     0m0.076s
```

##### ##Khost-based Rehosting

Command: `./scripts/coremark_test_khost.py`

You will be prompted:

```
Enable APT ? [Y/N] (default: Y):
```

Enter `Y` to rebuild Khost with APT enabled, `N` to rebuild khost with APT disabled.

Example output (**EXIT_OK** indicates that the run completed successfully, and the **passing time** line presents the result.):

```
=============== Round1 ===============
----------------- cjpeg -----------------
INFO  | khost: enable huge page
INFO  | khost: set mode to full
INFO  | khost: load config from experiments/performance/coremark-pro-mcu/cjpeg/config.yml
INFO  | khost: load single input form experiments/performance/coremark-pro-mcu/random.bin
INFO  | khost: set begin hook @ 0x08000e7a
INFO  | khost: set end hook @ 0x08000e8a
INFO  | khost: passing time: 2155.495000 ms
INFO  | khost: pc = 08000e8a, r0 = 00000000, r1 = 201ffd58, reg2 = 00000001, reg3 = 00000001
INFO  | khost: firmware EXIT_OK
```

##### ##Native Running

Command: `./scripts/coremark_test_native.sh`

Example output:

```
================== Round 0 ===================
--------------- experiments/performance/coremark-pro-linux/cjpeg-rose7-preset.elf -v0 -i100 -------------
...
Total time: 2119
-- Workload:cjpeg-rose7-preset=236760500
-- cjpeg-rose7-preset:time(ns)=2119
-- cjpeg-rose7-preset:contexts=1
-- cjpeg-rose7-preset:iterations=100
-- cjpeg-rose7-preset:time(secs)=   2.119
-- cjpeg-rose7-preset:secs/workload= 0.02119
-- cjpeg-rose7-preset:workloads/sec= 47.1921
-- Done:cjpeg-rose7-preset=236760500
```

#### On Simbench

##### ##QEMU-based Rehosting

Command: `./scripts/simbench_test_qemu.sh`

Example output:

```
################### Round 0 ########################
Analyzing experiments/performance/simbench-mcu/mem-hot/simbench.elf
Plugin watching ADDR1 = 0x800101c, ADDR2 = 0x8001020
[Start ] First exec at 0x800101c | Time: 1766385443610 ms
[Finish] First exec at 0x8001020 | Time: 1766385476356 ms
[Time Interval]: 32746 ms

real    0m32.843s
user    0m32.787s
sys     0m0.060s
```

##### ##Khost-based Rehosting

Command: `./scripts/simbench_test_khost.py`

You will be prompted:

```
Enable APT ? [Y/N] (default: Y):
```

Enter `Y` to rebuild Khost with APT enabled, `N` to rebuild khost with APT disabled.

Also, you will be prompted:

```
Move rehosting components out of KVM ? [Y/N] (default: N):
```

Enter `Y` to move rehosting components out of KVM (Khost-OUT).

Example output:

```
=============== Round 1 =================
-------------- mem-hot --------------
INFO  | khost: enable huge page
INFO  | khost: set mode to para
INFO  | khost: load config from experiments/performance/simbench-mcu/mem-hot/config.yml
INFO  | khost: load single input form experiments/performance/simbench-mcu/random.bin
INFO  | khost: set begin hook @ 0x0800101c
INFO  | khost: set end hook @ 0x08001020
INFO  | khost: passing time: 14561.005000 ms
INFO  | khost: pc = 08001020, reg2 = 00000000, reg3 = 00000000
INFO  | khost: firmware EXIT_OK
```

## Fuzzing Test

#### With AFL++ Backend

We use AFL++4.3 (b89727bea903aec80d003b6) as the default fuzzing backend.

Recompile :  `./clean.sh && ./build.sh`

Config core_pattern: `echo core | sudo tee /proc/sys/kernel/core_pattern`

Configure the CPU mode (skip this step if it is not supported on your system): `cd /sys/devices/system/cpu && echo performance | sudo tee cpu*/cpufreq/scaling_governor &&  cd -`

Start Fuzzing: `./scripts/fuzz_start.py item_of_firmware/all`

Fuzz one case: `./scripts/fuzz_start.py 01_PLC`. More firmware items are available under `experiments/fuzz`.

Fuzz all cases (to ensure that your hardware has enough cores): `./scripts/fuzz_start.py all`

Coverage: `./scripts/fuzz_replay.py item_of_firmware`

Replay & Debug: `./build/runner/khost-debug config_to_firmware input_file`

Count overhead from coverage collection: `./scripts/count_overhead_cov.py`

More usages, please use:  `./build/runner/khost-debug -h`

Crash cases are saved in `experiments/fuzz/item_of_firmware/crashes/`

To replay a crash, use `khost-debug`, which displays detailed information for a specific crash case. Khost provides three levels of debug modes, which can be selected using the `-l <level>` option:

- **error:** The default mode. Only error information is printed.
- **info:** Prints basic information about Khost’s configuration, such as the settings of all rehosting components.
- **debug:** Outputs detailed runtime information, including MMIO interactions. In this mode, Khost exits KVM frequently to print debug information and is therefore intended for debugging rather than fuzzing.


For example, `./build/runner/khost-debug experiments/fuzz/01_PLC/config.yml experiments/fuzz/01_PLC/crashes/id\:000074\,sig\:00\,src\:000414\,time\:9648045\,execs\:5625492\,op\:havoc\,rep\:13  -l debug -o all`

Example output:

```
INFO  | khost: set output level to error debug    (debug level)
INFO  | khost: enable output for all modules	  (enable debug for all rehosting modules)
INFO  | khost: load config from experiments/fuzz/01_PLC/config.yml
INFO  | khost: load single input form experiments/fuzz/01_PLC/crashes/id:000074,sig:00,src:000414,time:9648045,execs:5625492,op:havoc,rep:13
INFO  | loader: firmware path: experiments/fuzz/01_PLC/./PLC.bin
INFO  | loader: time measure begin: 0xffffffff     (Disable the overhead measurement module)
INFO  | loader: time measure end: 0xffffffff
INFO  | loader: basic blocks:
INFO  | loader: hook exit addrs:
INFO  | loader:     0x080040d4
INFO  | loader: hook nop addrs:
INFO  | loader: return zero addrs:
INFO  | loader: firmware info:
INFO  | loader:   load_base: 0x08000000
INFO  | loader:   num_irq: 106
INFO  | loader:   initial_sp: 0x20030000
INFO  | loader:   initial_pc: 0x08000d19
INFO  | loader:   firmware_code_size: 0x00006114
INFO  | loader:   addition_start: 0x07fac000
INFO  | host-board: crafting virtual board...
DEBUG | host-board: creating vcpu...
INFO  | host-board: set firmware entry: 0x08000d19
INFO  | host-board: set initial sp: 0x20030000
INFO  | host-board: set load base: 0x08000000
INFO  | host-board: set number of total irq: 106
INFO  | host-board: 0 incompatible instructions loaded to the runtime
INFO  | host-board: 100 fuzzware model loaded to the runtime
INFO  | host-board: set valid memory range: [0x40000000, 0x60000000)
INFO  | host-board: set valid memory range: [0xe0000000, 0xe0100000)
INFO  | host-board: set valid memory range: [0x20000000, 0x40000000)
INFO  | host-board: set valid memory range: [0x00000000, 0x20000000)
INFO  | host-board: virtual board successfully initialized
INFO  | host-board: set interrupt initial period: 0
DEBUG | host-cpu: reseting vcpu
DEBUG | host-cpu: reset finish
INFO  | host-board: running start
DEBUG | host-cpu: kvm enter, pc=0xf0004000
DEBUG | host-cpu: kvm exit, pc=0xf00041bc
GUEST | [guest-systick] set next irq to 0x000001F4
DEBUG | host-cpu: kvm enter, pc=0xf00041c0
DEBUG | host-cpu: kvm exit, pc=0xf00041bc
GUEST | [guest-systick] handle systick
GUEST | [guest-handler] MMIODataAbort: DFSR = 0x00000019, DFAR = 0x40023800, PC = 0x07FDBFB0, SP = 0x20030000, XPSR = 0x6000003F				(MMIO interactions)
DEBUG | host-cpu: kvm enter, pc=0xf00041c0
DEBUG | host-cpu: kvm exit, pc=0xf00041bc
.....

ERROR | host-cpu: Data Abort							(crash info. and vCPU's state)
ERROR | host-cpu: ====================== PC @ 0x0800090a ======================
ERROR | host-cpu: ARMv7-M Core Registers:
ERROR | host-cpu:    R0=0x00000000   R1=0x00000f72   R2=0x00000002
ERROR | host-cpu:    R3=0x00000020   R4=0x00000020   R5=0x08002fd3
ERROR | host-cpu:    R6=0x20202020   R7=0x00000000   R8=0x00000000
ERROR | host-cpu:    R9=0x00000000  R10=0x00000000  R11=0x00000000
ERROR | host-cpu:   R12=0x00000004   SP=0x2002ffc0   LR=0x00002020
ERROR | host-cpu: ARMv7-M CPU State:
ERROR | host-cpu:  CurrentMode=THREAD
ERROR | host-cpu:  N=0 Z=0 C=0 V=0 Q=0 GE=0x0 IPSR=0x00000000, EPSR=0x01000000
ERROR | host-cpu:  CONTROL .nPRIV=0 .SPSEL=0 .FPCA=1
ERROR | host-cpu:  PRIMASK=0x00000000 FAULTMASK=0x00000000 BASEPRI=0x00000000
ERROR | host-cpu: ARMv7-M FPU State:
ERROR | host-cpu:  S00=0x00000000 S01=0x00000000 S02=0x00000000 S03=0x00000000
ERROR | host-cpu:  S04=0x00000000 S05=0x00000000 S06=0x00000000 S07=0x00000000
ERROR | host-cpu:  S08=0x00000000 S09=0x00000000 S10=0x00000000 S11=0x00000000
ERROR | host-cpu:  S12=0x00000000 S13=0x00000000 S14=0x00000000 S15=0x00000000
ERROR | host-cpu:  S16=0x00000000 S17=0x00000000 S18=0x00000000 S19=0x00000000
ERROR | host-cpu:  S20=0x00000000 S21=0x00000000 S22=0x00000000 S23=0x00000000
ERROR | host-cpu:  S24=0x00000000 S25=0x00000000 S26=0x00000000 S27=0x00000000
ERROR | host-cpu:  S28=0x00000000 S29=0x00000000 S30=0x00000000 S31=0x00000000
ERROR | host-cpu: ARMv8-A Emulate State:
ERROR | host-cpu:  PSTATE=0x0000003f
ERROR | host-cpu:  PSTATE .E=0 .A=0 .I=0 .F=0 .T=1 .M=0x1f
ERROR | host-cpu: =============================================================
ERROR | host-cpu: Frimware Context:
ERROR | host-cpu:    PC=0x0800090a   XPSR=0x0000003f
ERROR | host-cpu:    R0=0x00000000   R1=0x00000f72   R2=0x00000002
ERROR | host-cpu:    R3=0x00000020   R4=0x00000020   R5=0x08002fd3
ERROR | host-cpu:    R6=0x20202020   R7=0x00000000   R8=0x00000000
ERROR | host-cpu:    R9=0x00000000  R10=0x00000000  R11=0x00000000
ERROR | host-cpu:   R12=0x00000004   SP=0x2002ffc0   LR=0x00002020
ERROR | host-cpu: =============================================================
ERROR | host-cpu: ESR = 0x0000080d
ERROR | host-board: running exit: EXIT_FIRMWARE_CRASH
INFO  | khost: firmware EXIT_CRASH
INFO  | khost: firmware EXIT_CRASH
========== Crash Trace ==========
Frimware Context: task_id=0 excp_id=0
   PC=0x0800090a   XPSR=0x0000003f
   R0=0x00000000   R1=0x00000f72   R2=0x00000002
   R3=0x00000020   R4=0x00000020   R5=0x08002fd3
   R6=0x20202020   R7=0x00000000   R8=0x00000000
   R9=0x00000000  R10=0x00000000  R11=0x00000000
  R12=0x00000004   SP=0x2002ffc0   LR=0x00002020
Stack Trace:									(The call stack at the time of the crash)
  [01] 0x0800090a: _ZN6Modbus11process_FC1EPth(+0x48)
  [02] 0x08000bd4: jpt_8000BBA(+0x16)
  [03] 0x08000c48: loop(+0x8)
  [04] 0x0800463e: main(+0xa)
  [05] 0x08000d4a: LoopFillZerobss(+0xe)
=================================
```

#### With LibAFL Backend

Install cargo: `apt install iproute2 cargo`
Install rustup: `curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`

Update cargo configuration to support ARMv7 by adding the following code to `~/.cargo/config`:

```
[target.armv7-unknown-linux-gnueabihf]
linker = "arm-linux-gnueabihf-gcc"
```

Compile: `cargo clean && cargo build --release `

Start Fuzzing: `./scripts/libafl_fuzz_start.py item_of_firmware/all`

Fuzz one case:  `./scripts/libafl_fuzz_start.py 01_PLC`

Fuzz all cases (to sure that your hardware has enough cores):  `./scripts/libafl_fuzz_start.py all`

**Note:** In the LibAFL version, the backend may continue running even after the program is forcefully terminated. Therefore, before starting a new round on the Raspberry Pi, please ensure that any running khost-libafl-fuzz backend processes are terminated.

Count Coverage: `./scripts/libafl_fuzz_replay.py item_of_firmware`

## Add New Firmware

Khost requires a `config.yml` file to describe the target firmware. This configuration file follows [Fuzzware’s definition and format](https://www.usenix.org/conference/usenixsecurity22/presentation/scharnowski). For reference implementations, please see the example configurations under `./experiments/fuzz/*/config.yml`.

```yaml
# Memory map for an ARM Cortex-M–based system
memory_map:
  mmio:
    base_addr: 0x40000000
    permissions: rw-
    size: 0x20000000
  system:
    base_addr: 0xe0000000
    permissions: rw-
    size: 0x00100000
  ram:
    base_addr: 0x20000000
    permissions: rw-
    size: 0x20000000
  flash:
    base_addr: 0x0
    permissions: r-x
    size: 0x20000000

# The basic block layout and symbol information can be generated using
# `./scripts/idapython_genconf.py` with IDA Pro.
#
# If `basic_blocks` is not provided, Khost will NOT rewrite the firmware
# binary, and execution will proceed without basic-block–level instrumentation.
basic_blocks:
  # Block entry address : block exit address
  0x80001ac: 0x80001b4
  0x80001b4: 0x80001b8
  0x80001b8: 0x80001be
  ...

symbols:
  # Symbol address (physical) : symbol name (the symbol name is optional and provided only for reference)
  0x8000000: g_pfnVectors
  0x80001ac: __do_global_dtors_aux
  0x80001d0: frame_dummy
  ...

# Path to the firmware binary, relative to the location of this config.yml file
firmware: ./PLC.bin

# MMIO models in Fuzzware format.
# This section may be left empty, in which case Fuzzware will automatically
# generate MMIO models during execution (if enabled).
mmio_models:
  ...
```

## MMIO Modeling Guidance

Khost uses **Fuzzware's Modeling Module** to generate MMIO models. Before running Khost, ensure the module has been installed in your system (see [Modeling Module](#modeling-module)) .

- For `khost`, enable Fuzzware-based MMIO modeling by passing the `-f` option at runtime.
- For `khost-afl-fuzz`, **automatic Fuzzware MMIO model generation is disabled by default**. To enable automatic MMIO modeling by adding `khost::Config::instance().option_set_auto_model(true);` to the `init_execution` function in `./src/engine/lib.cpp`.

## Run Fuzzware on ARM64

Please refer to `experiments/patches/fuzzware/README.md`

## Run HALucinator on ARM64

Please refer to `experiments/patches/halfuzz/README.md`
