# Simulation Bridge

`sim_bridge.py` is part of this repository and is cloned with the simulation workspace. It is a
small WSL-side HTTP service for the real vehicle: the vehicle sends `POST /start` with the actual
item/category, then polls `GET /status`; the bridge launches `task3_execute.launch` and reports
the latched `/sim_task3/done` result.

It does not alter the planner, URDF, world, or task route. It uses only Python 3 standard-library
modules and must be started only after `task3_prepare.launch` is ready on the **separate** simulation
Master `127.0.0.1:11312`.

## Start

In one persistent WSL terminal, start the simulation prepare launch and wait for `/map`:

```bash
cd ~/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://127.0.0.1:11312
export ROS_IP=127.0.0.1
roslaunch car3 task3_prepare.launch gui:=true rviz:=true
```

In another persistent WSL terminal:

```bash
cd ~/smartcar2026-simulation
source /opt/ros/noetic/setup.bash
source devel/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://127.0.0.1:11312
export ROS_IP=127.0.0.1
python3 bridge/sim_bridge.py
```

Wait for `SIMULATION_BRIDGE_READY` and `state=waiting` before starting the vehicle mission. The
default HTTP port is `11313`; Windows Firewall must allow TCP 11313 from `LocalSubnet` on trusted
networks. The real vehicle never connects to 11312; it reaches the bridge at
`http://<WSL_LAN_IP>:11313`.

## API and lifecycle

```text
waiting --POST /start--> running --/sim_task3/done=True--> done
                             |--roslaunch error or timeout--> failed
```

`POST /start` body example: `{"item_name":"苹果","category":"食品"}`.
`GET /status` reports `waiting`, `running`, `done`, or `failed`. One bridge instance serves one
simulation run; after `done` or `failed`, stop it with Ctrl-C and start a new instance.

The bridge stores its task output as `bridge/task3_run_*.log`; these runtime logs are ignored by
Git. For the full new-computer procedure, WSLg COPY MODE checks, firewall commands, and shutdown
sequence, see the real-vehicle repository's `docs/new-computer-gui-simulation-mission.md`.
