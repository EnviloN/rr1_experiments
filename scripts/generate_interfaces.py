#!/usr/bin/env python3
import argparse
import os


def generate_interfaces(path, start=1, end=20):
    for i in range(start, end + 1):
        payload_size = 2**i
        file_name = f"Experiment{payload_size}B.msg"
        file_path = os.path.join(path, file_name)
        
        with open(file_path, 'w') as f:
            f.write(f"float64 timestamp\n")
            f.write(f"byte[{payload_size}] payload")
    print(f"Generated {end - start + 1} service interfaces in {path}.")

def main():
    parser = argparse.ArgumentParser(
        description="Generate service interface definition files with increasing payload sizes."
        )
    parser.add_argument("path", type=str, help="Path to the directory where the files should be generated.")
    parser.add_argument("start", type=int, default=1, help="Starting power of two (inclusive).")
    parser.add_argument("end", type=int, default=20, help="Ending power of two (inclusive).")
    args = parser.parse_args()

    generate_interfaces(args.path, args.start, args.end)


if __name__ == "__main__":
    main()