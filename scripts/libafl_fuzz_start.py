#! /usr/bin/env python3
import os
import sys
from typing import Optional
from datetime import datetime, timezone, timedelta
import subprocess, time

CORE_START = 1
PORT_START = 1357
OFFSET = 12

FILE_PATH = os.path.dirname(__file__)
FUZZ_BASE_PATH = os.path.join(FILE_PATH, "../experiments/libafl_fuzz")
FUZZER_PATH = os.path.join(FILE_PATH, "../target/release/khost-libafl-fuzz")

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

FUZZ_WORK_DIR_NAME = "LibAFLWorkspace"


def get_free_cpu():

    cmd = "ps -eLo pid,psr,comm | grep -E 'afl-fuzz|khost-libafl-fuzz' | awk -F ' ' '{print $2}'"
    try:
        res = subprocess.run(cmd, shell=True, check=True, text=True, capture_output=True)
        cpu_lists = []
        for line in res.stdout.splitlines():
            if len(line) != 0:
                cpu_lists.append(int(line))
        for i in range(1, 64):
            if i not in cpu_lists:
                return i

        return -1
    except Exception as e:
        print(f"{e}")
        return -1

def get_free_port():

    cmd = "ss -tulnp | awk -F ' ' '{print $5}' | awk -F ':' '{print $2}'"
    try:
        res = subprocess.run(cmd, shell=True, check=True, text=True, capture_output=True)
        port_lists = []
        for line in res.stdout.splitlines():
            if len(line) != 0:
                port_lists.append(int(line))
        for i in range(1337, 1437):
            if i not in port_lists:
                return i

        return -1
    except Exception as e:
        print(f"{e}")
        return -1


def fuzz_item(item_name: str, timeout: Optional[str]):
    if item_name not in fuzz_items:
        print(f"fuzz item {item_name} not in item list")
        return

    # create item dir
    item_fuzz_path = os.path.join(FUZZ_BASE_PATH, item_name)
    if not os.path.exists(item_fuzz_path):
        os.mkdir(item_fuzz_path)
    os.chdir(item_fuzz_path)
    
    cur_datetime = datetime.now(timezone(timedelta(hours=8))).strftime("%Y-%m-%d-%H-%M-%S")
    fuzz_work_dir = os.path.join(FUZZ_BASE_PATH, item_name, f"{FUZZ_WORK_DIR_NAME}-{cur_datetime}")
    if os.path.exists(fuzz_work_dir):
        print("fuzz workspace not empty")
        return
    os.mkdir(fuzz_work_dir)

    # craft fuzz command
    cd_cmd = f"cd {fuzz_work_dir}"

    fuzz_cmd = ""
    if timeout is not None:
        fuzz_cmd += f"timeout {timeout} "
    else:
        fuzz_cmd += f"timeout 24h "

    fuzz_cmd += FUZZER_PATH
    fuzz_cmd += f" -b {os.path.join('..', f'{FUZZ_BASE_PATH}/{item_name}/config.yml')}"
    fuzz_cmd += f" -i {os.path.join('..', 'corpus')}"
    fuzz_cmd += " -f"
    # global CORE_START, OFFSET
    # fuzz_cmd += f" -c {CORE_START + OFFSET}"
    # fuzz_cmd += f" -p {PORT_START + OFFSET}"
    # OFFSET += 1
    cpu_id = get_free_cpu()
    port_id = get_free_port()
    fuzz_cmd += f" -c {cpu_id}"
    fuzz_cmd += f" -p {port_id}"
    fuzz_cmd += " 2>&1 | tee output.log"
    
    print("Start fuzzing:")
    print(cd_cmd)
    print(fuzz_cmd)
    
    # start tmux
    session_name = f"khost_libafl_{item_name}"
    if os.system(f"tmux new -d -s {session_name}") != 0:
        for i in range(1, 100):
            if os.system(f"tmux new -d -s {session_name}_{i}") == 0:
                session_name = f"{session_name}_{i}"
                break
        if session_name == f"khost_libafl_{item_name}":
            print("failed to create session")
            exit(1)
    # change the directory
    os.system(f"tmux send-keys -t {session_name} '{cd_cmd}' Enter")
    # run the command
    os.system(f"tmux send-keys -t {session_name} '{fuzz_cmd}' Enter")
    print(f"running in tmux session {session_name}")


def fuzz_all(timeout):
    for item in fuzz_items:
        fuzz_item(item, timeout)
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
