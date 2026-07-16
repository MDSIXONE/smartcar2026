# Set the WSL ROS Master rule

## Purpose

Record the required ROS master topology for the vehicle: the ROS Master runs
on the local WSL Ubuntu 20.04 host, not on the vehicle.

## Changed files

- `AGENTS.md`
  - adds the mandatory master endpoint for vehicle nodes, diagnostics, and
    launch commands.
- `docs/operations.md`
  - makes all documented vehicle launch, mapping, and RViz examples use the
    same endpoint.

## Required environment

After sourcing ROS and the workspace on the vehicle, set:

```bash
export ROS_IP=192.168.8.231
export ROS_MASTER_URI=http://192.168.8.197:11311
```

Do not start or use a vehicle-local `roscore` at `192.168.8.231:11311`.

## Validation

- All `ROS_MASTER_URI` examples in `docs/operations.md` point to
  `192.168.8.197:11311`.
- No vehicle process was started, stopped, or reconfigured while recording
  this rule.
