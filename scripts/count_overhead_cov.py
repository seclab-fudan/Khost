#! /usr/bin/env python3

import os, sys, subprocess, re

dir_khost = os.path.abspath('.')

path_khost = os.path.join(dir_khost, "build/runner/khost")
path_config_origin = os.path.join(dir_khost, "experiments/fuzz/overhead_coverage/config.yml")
path_config_nocov  = os.path.join(dir_khost, "experiments/fuzz/overhead_coverage/config-nocov.yml")
path_input = os.path.join(dir_khost, "experiments/fuzz/overhead_coverage/base_input")


def extract_real_time(output_cmd):

    match = re.search(r"([\d.]+)\s*ms", output_cmd)
    if match:
        total_seconds = float(match.group(1))
        return total_seconds
    else:
        return 0



def count_overhead_coverage():

    try:
        num_round = 3
        times_one_round = 100
        time1_total = 0
        time2_total = 0
        print(f"This benchmark runs 3 x 100 iterations and may take some time. Please wait for it to complete.")
        print(f"Starting test....")
        for r in range(0, num_round):
            for i in range(0, times_one_round):
                time1 = 0
                time2 = 0
                try: 
                    cmd1 = f"{path_khost} {path_config_nocov} {path_input}"
                    res1 = subprocess.run(cmd1, shell=True, text=True, capture_output=True)
                    if res1.stdout:
                        time1 = extract_real_time(res1.stdout)
                    # print(cmd1) 
                    cmd2 = f"{path_khost} {path_config_origin} {path_input}"
                    res2 = subprocess.run(cmd2, shell=True, text=True, capture_output=True)
                    if res2.stdout:
                        time2 = extract_real_time(res2.stdout)
                except Exception as e:
                    print(f"Fail in round {i + 1}, {e}", flush=True)
                    pass
                if time1 != 0 and time2 != 0:
                    time1_total += time1
                    time2_total += time2
            
            time_overhead = (time2_total - time1_total) / time1_total * 100
            print(f"Round {r + 1}: time1_total={time1_total}, time2_total={time2_total}, time_overhead={time_overhead}%")
    except Exception as e:
        print(f"Error: {e}", flush=True)



if __name__ == "__main__":

    count_overhead_coverage()
