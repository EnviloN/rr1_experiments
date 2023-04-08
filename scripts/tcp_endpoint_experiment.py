#!/usr/bin/env python3
import subprocess, re, signal, time

expected_output = re.compile(r".*Experiment finished\.")
time_delta = 50

for i in range(1, 22):
    if i >=21:
        time_delta = 1000
    elif i >=19:
        time_delta = 500
    elif i >= 17:
        time_delta = 100
    cmd = ["ros2", "launch", "rr1_experiments", "tcp_endpoint_experiment_launch.py",
           "payload_size:={}".format(i), "time_delta:={}".format(time_delta)]
    
    print("Running experiment with payload size {} and time delta {}".format(2**i, time_delta))
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    while True:
        output = p.stdout.readline().decode().strip()
        match = expected_output.search(output)
        if match:
            p.send_signal(signal.SIGINT)
            break

time.sleep(10)