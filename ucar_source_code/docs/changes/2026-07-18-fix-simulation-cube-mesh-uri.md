# 2026-07-18 Fix Simulation Cube Mesh URI

## Purpose

Make the Task 3 cube visuals portable across local WSL deployments.

## Changed files

- `simulation/src/car3/models/cube/model_0.sdf`
- `simulation/src/car3/models/cube/model_1.sdf`
- `simulation/src/car3/models/cube/model_2.sdf`
- `simulation/src/car3/test/test_cube_mesh_uri.py`
- `simulation/src/car3/CMakeLists.txt`
- `docs/operations.md`

## Cause and fix

The cube spawner successfully created Gazebo entities, but each SDF visual mesh
used the absolute path from the original `/home/car/gazebo_ws_v3_local`
workspace. That path does not exist in the local WSL deployment, leaving the
collision bodies without visible meshes. The visual URIs now use
`model://cube/meshes/cube_*.obj`. The project `GAZEBO_MODEL_PATH` is now set
before `gzserver` starts, so the service that resolves the URI receives it.

## Verification

- Added a standard-library CTest regression check that rejects absolute cube
  mesh URIs, verifies the referenced mesh files exist, and verifies the launch
  sets `GAZEBO_MODEL_PATH` before starting Gazebo.
- The regression script passes in the synchronized WSL workspace.
- `task3_prepare.launch` is restarted and `/gazebo/get_world_properties` is
  checked for `cube_0`, `cube_1`, and `cube_2`.

## Known limitations

- The objects are physically 4 cm cubes, so they remain small at a wide
  Gazebo camera zoom even when their visual meshes load correctly.
