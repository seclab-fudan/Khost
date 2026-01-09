#! /usr/bin/env python3
import os
import sys

FILE_PATH = os.path.dirname(__file__)
FUZZ_BASE_PATH = os.path.join(FILE_PATH, "../experiments/libafl_fuzz")
REPLAYER_PATH = os.path.join(FILE_PATH, "../build/runner/khost-debug")

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

def replay(config_path: str, queue_dir: str, crash_dir: str, hang_dir: str):
    fuzz_dir, _ = os.path.split(config_path)
    ground_truth_path = os.path.join(fuzz_dir, "..", "valid_basic_blocks.txt")
    rewrite_result_path = os.path.join(fuzz_dir, "rewrite_result.txt") 
    if os.path.exists(rewrite_result_path):
        os.remove(rewrite_result_path)
        
    cmd = REPLAYER_PATH
    cmd += " -t 10"
    cmd += " -b"
    cmd += " -c " + ground_truth_path
    cmd += " -r " + rewrite_result_path
    cmd += " " + config_path
    cmd += " " + queue_dir
    cmd += " " + crash_dir
    # cmd += " " + hang_dir

    path_stats = os.path.join(os.path.dirname(queue_dir), "output.log")
    with open(path_stats, 'r') as ff:
        lines = ff.readlines()
        for line in reversed(lines):
            if 'GLOBAL' in line and 'exec/sec' in line:
                exec_per_sec = line.split(':')[6].strip()
                break
    print(f"cmd: {cmd}")
    os.system(cmd)
    print(f"exec_per_sec: {exec_per_sec}")
    
def main():
    if len(sys.argv) != 2:
        print("usage: fuzz_replay name_of_the_item")
        exit(1)

    if not os.path.exists(REPLAYER_PATH):
        print("replayer not found, try to compile replayer")
        if os.system(f"{FILE_PATH}/../build.sh") != 0:
            print("failed to compile replayer")
            exit(1)
    
    if sys.argv[1] not in fuzz_items:
        print(f"fuzz item {sys.argv[1]} not in item list")
        exit(1)
    
    fuzz_dirs = sorted(list(filter(lambda x: x.startswith(FUZZ_WORK_DIR_NAME), os.listdir(f"{FUZZ_BASE_PATH}/{sys.argv[1]}"))))
    if len(fuzz_dirs) == 1:
        replay(
            os.path.join(FUZZ_BASE_PATH, sys.argv[1], fuzz_dirs[0], "config.yml"),
            os.path.join(FUZZ_BASE_PATH, sys.argv[1], fuzz_dirs[0], "queue"),
            os.path.join(FUZZ_BASE_PATH, sys.argv[1], fuzz_dirs[0], "crashes"),
            os.path.join(FUZZ_BASE_PATH, sys.argv[1], fuzz_dirs[0], "hangs")
        )
        return
    if len(fuzz_dirs) == 0:
        print(f"no fuzz workspace found for {sys.argv[1]}")
        return

    print(f"multi fuzz workspaces found for {sys.argv[1]}:")
    for id in range(len(fuzz_dirs)):
        print(f"  {id+1}: {fuzz_dirs[id]}")
    id = input("select id> ")
    if id.isalnum() and int(id) >= 1 and int(id) <= len(fuzz_dirs):
        replay(
            os.path.join(FUZZ_BASE_PATH, sys.argv[1], fuzz_dirs[int(id)-1], "config.yml"),
            os.path.join(FUZZ_BASE_PATH, sys.argv[1], fuzz_dirs[int(id)-1], "queue"),
            os.path.join(FUZZ_BASE_PATH, sys.argv[1], fuzz_dirs[int(id)-1], "crashes"),
            os.path.join(FUZZ_BASE_PATH, sys.argv[1], fuzz_dirs[int(id)-1], "hangs")
        )
        return
    else:
        print("invalid id")

if __name__ == '__main__':
    main()
