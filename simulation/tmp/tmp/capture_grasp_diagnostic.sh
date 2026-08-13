#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
source /home/car/smartcar2026-simulation/devel/setup.bash
exec python3 /mnt/d/WORK/ALLCODE/smartcar2026/simulationforreal/simulation/smartcar2026-simulation-yolo-mode/tmp/grasp_float_diagnostic.py
