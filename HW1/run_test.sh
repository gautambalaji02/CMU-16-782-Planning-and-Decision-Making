#!/bin/bash
set -euo pipefail

if [[ $# -ne 1 || ! $1 =~ ^map[1-9]$ ]]; then
    echo "Usage: $0 map<digit> (where digit is 1-9)" >&2
    exit 1
fi

./build/run_test "$1.txt"

python3 scripts/plot_static.py maps/"$1".txt output/robot_trajectory.txt scripts/"$1".png
