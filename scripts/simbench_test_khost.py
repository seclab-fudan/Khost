#! /usr/bin/env python3
import os, multiprocessing, subprocess, re

base_path = f"{os.path.dirname(__file__)}/../experiments/performance/simbench-mcu"
proj_path = f"{os.path.dirname(__file__)}/../"

testcases = [
    "mem-hot",
    "mem-cold",
    "coprocessor",
    "device-access",
    "dfault",
    "ifault",
    "syscall",
    "undef",
    "inter-page-direct",
    "inter-page-indirect",
    "same-page-direct",
    "same-page-indirect",
    "large-blocks",
    "small-blocks"
]

RUN_COUNT = 3

def compile_with_option(enable_apt: bool):
    build_path = os.path.join(proj_path, "build")
    os.system("bash " + os.path.join(proj_path, "clean.sh"))
    os.mkdir(build_path)
    os.chdir(build_path)
    max_cores = multiprocessing.cpu_count() - 1
    if enable_apt:
        os.system("cmake -DCMAKE_BUILD_TYPE=Release ../src")
        os.system(f"make -j{max_cores}")
        os.system("cp ./runner/khost /tmp/khost-apt")
    else:
        os.system("cmake -DUSE_APT=OFF -DCMAKE_BUILD_TYPE=Release ../src")
        os.system(f"make -j{max_cores}")
        os.system("cp ./runner/khost /tmp/khost-napt")

def clean_with_option(enable_apt: bool):
    if enable_apt:
        os.system("rm /tmp/khost-apt")
    else:
        os.system("rm /tmp/khost-napt")

def is_raspberry_pi():
    try:
        with open("/proc/cpuinfo", "r") as f:
            cpuinfo = f.read().lower()
        return "raspberry pi" in cpuinfo or "bcm" in cpuinfo
    except FileNotFoundError:
        return False


def main():

    # Due to the intermediate files are different for different mode, so we must recompile.
    enable_apt = input("Enable APT ? [Y/N] (default: Y): ")
    enable_apt = False if enable_apt == "N" else True

    print(f"Compiling kvm runner with APT {'enabled' if enable_apt else 'disabled' }")
    compile_with_option(enable_apt)
    flag_out = "full"
    if enable_apt:
        enable_out = input("Move rehosting components out of KVM ? [Y/N] (default: N): ")
        flag_out = "full" if enable_apt == "N" else "para"

    runner = "/tmp/khost-apt" if enable_apt else "/tmp/khost-napt"
    
    os.chdir(proj_path)
    print("Testing...")
    results = {}
    for testcase in testcases:
        results[testcase] = []
    for i in range(0, RUN_COUNT):
        print(f"=============== Round {i+1} =================")
        for testcase in testcases:
            print(f"-------------- {testcase} --------------")
            config_path = base_path + "/" + testcase + "/config.yml"
            input_path = base_path + "/random.bin"
            if is_raspberry_pi():
                cmd = f"{runner} -m {flag_out} {config_path} {input_path}"
            else:
                cmd = f"{runner} -p -m {flag_out} {config_path} {input_path}"
            # print(f"CMD: {cmd}")
            res = subprocess.run(cmd, shell=True, check=True, text=True, capture_output=True)
            print(res.stdout)
            match = re.search(r'passing time:\s*([\d.]+)', res.stdout)
            if match:
                interval = float(match.group(1))
            results[testcase].append(interval)
    
    clean_with_option(enable_apt)

    print("="*50)
    print(f"%-20s " % "Test Case", end = "")
    for i in range(0, RUN_COUNT - 1):
        title = f"Round{i + 1}"
        print(f"%-10s " % title, end = "")
    title = f"Round{RUN_COUNT}"
    print(f"%-10s" % title)
    for testcase in testcases:
        rest = results[testcase]
        print(f"%-20s " % testcase, end = "")
        for i in range(0, RUN_COUNT - 1):
            print(f"%-10.3f " % rest[i], end = "")
        print(f"%-10.3f" % rest[RUN_COUNT - 1])
    print("="*50)

    print("All tests done!")

if __name__ == '__main__':
    main()
