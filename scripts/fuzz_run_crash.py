#! /usr/bin/env python3
import os
import io
import re
import sys
import subprocess

FILE_PATH = os.path.dirname(__file__)
FUZZ_BASE_PATH = os.path.join(FILE_PATH, "../experiments/fuzz")
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
    "12_nuttx-nsh"
]

FUZZ_WORK_DIR_NAME = "AFLWorkspace"

def run_crash(config_path: str, input_path: str):
    os.system(f"{REPLAYER_PATH} -p -l error -o cpu {config_path} {input_path}")

def in_crash_dir(config_path: str, crash_dir: str):
    crash_inputs = sorted(list(filter(lambda x: x.startswith("id"), os.listdir(crash_dir))))
    if len(crash_inputs) == 1:
        run_crash(config_path, os.path.join(crash_dir, crash_inputs[0]))
        return
    if len(crash_inputs) == 0:
        print(f"no crash found for {sys.argv[1]}")
        return

    print(f"multi crashes found for {sys.argv[1]}:")
    for id in range(len(crash_inputs)):
        print(f"  {id+1}: {crash_inputs[id]}")
    id = input("select id> ")
    if id.isalnum() and int(id) >= 1 and int(id) <= len(crash_inputs):
        run_crash(config_path, os.path.join(crash_dir, crash_inputs[int(id)-1]))
        return
    else:
        print("invalid id")

def main():
    if len(sys.argv) != 2:
        print("usage: fuzz_run_crash name_of_the_item")
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
        in_crash_dir(
            f"{FUZZ_BASE_PATH}/{sys.argv[1]}/{fuzz_dirs[0]}/default/config.yml",
            f"{FUZZ_BASE_PATH}/{sys.argv[1]}/{fuzz_dirs[0]}/default/crashes"
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
        in_crash_dir(
            f"{FUZZ_BASE_PATH}/{sys.argv[1]}/{fuzz_dirs[int(id)-1]}/default/config.yml",
            f"{FUZZ_BASE_PATH}/{sys.argv[1]}/{fuzz_dirs[int(id)-1]}/default/crashes"
        )
        return
    else:
        print("invalid id")

if __name__ == '__main__':
    main()
