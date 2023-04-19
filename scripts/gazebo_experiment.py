#!/usr/bin/env python3
import argparse, subprocess, os
from ament_index_python.packages import get_package_share_directory

EXPERIMENTS_PKG = "rr1_experiments"

SCRIPT_PATH = os.path.join(get_package_share_directory(EXPERIMENTS_PKG),
                           'scripts', 'generate_gazebo_world.py')
ROS_ECHO_CMD = ["ros2", "topic", "echo", "/performance_metrics", "--once"]

def generate_world(args):
    cmd = ["python3", SCRIPT_PATH, str(args.cnt)]
    if args.dynamic:
        cmd.append("--dynamic")
    p = subprocess.Popen(cmd)

def run_gazebo():
    cmd = ["ros2", "launch", EXPERIMENTS_PKG, "gazebo_experiment.launch.py"]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("cnt", type=int, default=10)
    parser.add_argument("--dynamic", action='store_true')
    args = parser.parse_args()

    generate_world(args)
    run_gazebo()

if __name__ == "__main__":
    main()