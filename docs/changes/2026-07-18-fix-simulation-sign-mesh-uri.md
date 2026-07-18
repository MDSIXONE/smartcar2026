# 2026-07-18 Fix Simulation Scene Label Mesh URI

## Purpose

Restore the visible Food, Daily Necessities, and Electronics scene labels in
the local WSL Gazebo simulation.

## Changed files

- `simulation/src/car3/world/math.world`
- `simulation/src/car3/models/sign/model.config`
- `simulation/src/car3/models/sign/model.sdf`
- `simulation/src/car3/test/test_sign_mesh_uri.py`
- `simulation/src/car3/CMakeLists.txt`
- `docs/operations.md`

## Cause and fix

The three wall-label meshes used the original machine's absolute
`/home/car/gazebo_ws_v3_local` path. It does not exist in the local WSL
deployment. The sign resource folder is now a valid Gazebo model and the
world uses portable `model://sign/meshes/...` URIs.

## Verification

- Added a standard-library regression test that requires a valid sign model
  directory and portable URI for each wall label.
- The regression script passes in the synchronized WSL workspace.
- Restart Gazebo; visual verification must confirm all three labels are in
  the active camera view.

## Known limitations

- Label visibility still depends on the active Gazebo camera angle and zoom.
