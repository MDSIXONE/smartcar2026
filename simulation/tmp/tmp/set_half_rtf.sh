#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
rosservice call /gazebo/set_physics_properties \
  "time_step: 0.01
max_update_rate: 50.0
gravity: {x: 0.0, y: 0.0, z: -9.8}
ode_config:
  auto_disable_bodies: false
  sor_pgs_precon_iters: 0
  sor_pgs_iters: 50
  sor_pgs_w: 1.3
  sor_pgs_rms_error_tol: 0.0
  contact_surface_layer: 0.001
  contact_max_correcting_vel: 100.0
  cfm: 0.0
  erp: 0.2
  max_contacts: 20"
rosservice call /gazebo/get_physics_properties
