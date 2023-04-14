#!/usr/bin/env python3
import gpustat, psutil, time

TIME_DELTA = 1
while True:
    gpu_stats = gpustat.new_query().gpus[0]
    gpu_temp = gpu_stats.temperature
    cpu_temp = psutil.sensors_temperatures()["k10temp"][0].current
    print("-------------------------")
    print("GPU: {}".format(gpu_temp))
    print("CPU: {}".format(cpu_temp))
    print("-------------------------")
    time.sleep(TIME_DELTA)
