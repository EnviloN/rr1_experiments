#!/usr/bin/env python3
import argparse, time, subprocess, signal, sys
import gpustat, psutil

ROS_ECHO_CMD = ["ros2", "topic", "echo", "/performance_metrics", "--once"]
TIME_DELTA = 0.05

logfile = "/home/envilon/dev_ws/src/rr1_experiments/analysis/data/Unity/static_test.csv"
file = open(logfile, "a")

# define a function to be called when the signal is received
def signal_handler(signal, frame):
    print("Closing File!")
    file.close()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def get_gpu_usage():
    gpu_stats = gpustat.new_query().gpus[0]
    return gpu_stats.utilization, gpu_stats.temperature, gpu_stats.memory_used    

def get_real_time_factor():
    result = subprocess.run(ROS_ECHO_CMD, capture_output=True, text=True)
    return result.stdout.splitlines()[-3].split()[1]

def log_performance(args):
    counter = 0
    input("Press any key to start logging...")
    while True:
        gpu = gpustat.new_query().gpus[0]
        cpu = psutil.cpu_percent(percpu=True)
        cpu_temp = psutil.sensors_temperatures()["k10temp"][0].current
        ram = psutil.virtual_memory()
        # rtf = get_real_time_factor()

        print("[{}] -> GPU: {}".format(counter, gpu.utilization))
        line = '\n{},{},"{}",{},{},{},{},{}'.format(
            "dynamic" if args.dynamic else "static", args.cnt,
            # rtf,
            cpu, cpu_temp,
            gpu.utilization, gpu.temperature,
            ram.used, gpu.memory_used
        )
        # if counter != 0:
        file.write(line)
        counter += 1
        if counter >= 20:
            return
        time.sleep(TIME_DELTA)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("cnt", type=int, default=10)
    parser.add_argument("--dynamic", action='store_true')
    args = parser.parse_args()
    log_performance(args)

if __name__ == "__main__":
    main()