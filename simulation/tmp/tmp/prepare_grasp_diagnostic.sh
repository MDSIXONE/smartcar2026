#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
source /home/car/smartcar2026-simulation/devel/setup.bash
exec roslaunch car3 task3_prepare.launch gui:=false rviz:=false
