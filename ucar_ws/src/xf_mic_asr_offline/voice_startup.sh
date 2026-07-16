#!/bin/sh

source /opt/ros/melodic/setup.bash
source /home/ucar/ucar_ws/devel/setup.bash
roslaunch xf_mic_asr_offline mic_init.launch
 
exit 0
