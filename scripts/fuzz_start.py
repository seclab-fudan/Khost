#! /usr/bin/env python3
import os
import sys
from typing import Optional
from datetime import datetime, timezone, timedelta

FILE_PATH = os.path.dirname(__file__)
FUZZ_BASE_PATH = os.path.join(FILE_PATH, "../experiments/fuzz")
FUZZER_PATH = os.path.join(FILE_PATH, "../build/fuzzer/khost-afl-fuzz")

fuzz_items = [
    "01_PLC",
    "02_Drone",
    "03_Heat_Press",
    "04_stm32_sine",
    "05_LibJPEG_Decoding",
    "06_avem",
    "07_zephyr-modbus",
    "08_atmel_6lowpan_udp_rx",
    "09_stm32_tcp_echo_client",
    "10_stm32_tcp_echo_server",
    "11_BlueMicro_BLE",
    "12_nuttx-nsh",
]

FUZZ_WORK_DIR_NAME = "AFLWorkspace"

def fuzz_item(item_name: str, timeout: Optional[str]):
    if item_name not in fuzz_items:
        print(f"fuzz item {item_name} not in item list")
        return
    
    cur_datetime = datetime.now(timezone(timedelta(hours=8))).strftime("%Y-%m-%d-%H-%M-%S")
    fuzz_work_dir = os.path.join(FUZZ_BASE_PATH, item_name, f"{FUZZ_WORK_DIR_NAME}-{cur_datetime}")
    if os.path.exists(fuzz_work_dir):
        print("fuzz workspace not empty")
        return
    os.mkdir(fuzz_work_dir)

    cmd = f"AFL_AUTO_AFFINITY=1 AFL_SKIP_BIN_CHECK=1 AFL_NO_FORKSRV=1 AFL_INPUT_LEN_MAX=10240 AFL_SKIP_CRASHES=1 AFL_I_DONT_CARE_ABOUT_MISSING_CRASHES=1"
    if timeout is not None:
        cmd += f" timeout -s 9 {timeout}"
    else:
        cmd += f" timeout -s 9 24h"

    cmd += f" {FUZZER_PATH}"
    cmd += f" -i {FUZZ_BASE_PATH}/{item_name}/corpus"
    cmd += f" -o {fuzz_work_dir}"
    cmd += f" -t 10000"
    cmd += f" -T {item_name}"
    cmd += f" -- {FUZZ_BASE_PATH}/{item_name}/config.yml"
    
    print(cmd)
    
    session_name = f"fuzz_{item_name}"
    if os.system(f"tmux new -d -s {session_name}") != 0:
        for i in range(1, 100):
            if os.system(f"tmux new -d -s {session_name}_{i}") == 0:
                session_name = f"{session_name}_{i}"
                break
        if session_name == f"fuzz_{item_name}":
            print("failed to create session")
            exit(1)
    os.system(f"tmux send-keys -t {session_name} '{cmd}' Enter")
    print(f"running in tmux session {session_name}")

def fuzz_all(timeout):
    import time
    for item in fuzz_items:
        fuzz_item(item, timeout)
        # wait for afl++ finish binding the CPU
        # without delay we might have 2 fuzzers on the same core
        time.sleep(10)

def main():
    if len(sys.argv) < 2 or len(sys.argv) > 3:
        print("fuzz one target:")
        print("  usage: fuzz_start name_of_the_item [timeout]")
        print("fuzz all targets: ")
        print("  usage: fuzz_start all [timeout]")
        exit(1)

    if not os.path.exists(FUZZER_PATH):
        print("fuzzer not found, try to compile fuzzer")
        if os.system(f"{FILE_PATH}/../build.sh") != 0:
            print("failed to compile fuzzer")
            exit(1)

    timeout = None
    if len(sys.argv) == 3:
        timeout = sys.argv[2]
    
    if sys.argv[1] == "all":
        fuzz_all(timeout)
    else:
        fuzz_item(sys.argv[1], timeout)

if __name__ == "__main__":
    main()
