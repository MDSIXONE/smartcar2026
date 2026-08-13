#!/usr/bin/env python3
"""Load official scene-label meshes when their legacy absolute URI is absent.

The distributed world stays byte-for-byte unchanged.  On machines that do not
have the original /home/ucar/gazebo_ws path, this node spawns visual-only
overlays from the same OBJ resources through the package's model:// path.
"""

import copy
import math
import os
import posixpath
from collections import namedtuple
from urllib.parse import unquote, urlparse
from xml.etree import ElementTree


Overlay = namedtuple("Overlay", "model_name pose_text sdf_xml mesh_name")
COMPAT_SUFFIX = "__resource_compat"


def _file_uri_path(uri):
    parsed = urlparse(uri)
    if parsed.scheme != "file":
        return None
    return unquote(parsed.path)


def build_missing_sign_overlays(world_xml, resource_exists=os.path.exists):
    """Return portable overlays for unresolved official wall-sign resources."""
    root = ElementTree.fromstring(world_xml)
    overlays = []
    for model in root.findall(".//model"):
        source_name = model.get("name", "")
        if not source_name.startswith("wall_"):
            continue
        uri_node = model.find(".//visual/geometry/mesh/uri")
        if uri_node is None or not uri_node.text:
            continue
        source_path = _file_uri_path(uri_node.text.strip())
        if source_path is None or resource_exists(source_path):
            continue

        mesh_name = posixpath.basename(source_path)
        portable_model = copy.deepcopy(model)
        portable_model.set("name", source_name + COMPAT_SUFFIX)
        pose_node = portable_model.find("pose")
        pose_text = (
            pose_node.text.strip()
            if pose_node is not None and pose_node.text
            else "0 0 0 0 0 0"
        )
        if pose_node is not None:
            portable_model.remove(pose_node)
        portable_uri = portable_model.find(
            ".//visual/geometry/mesh/uri"
        )
        portable_uri.text = "model://sign/meshes/" + mesh_name
        sdf_root = ElementTree.Element("sdf", {"version": "1.7"})
        sdf_root.append(portable_model)
        overlays.append(
            Overlay(
                model_name=source_name + COMPAT_SUFFIX,
                pose_text=pose_text,
                sdf_xml=ElementTree.tostring(
                    sdf_root, encoding="unicode"
                ),
                mesh_name=mesh_name,
            )
        )
    return overlays


def _pose_from_text(pose_text, Pose, Point, Quaternion):
    values = [float(value) for value in pose_text.split()]
    if len(values) != 6:
        raise ValueError("expected x y z roll pitch yaw pose")
    x, y, z, roll, pitch, yaw = values
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5
    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)
    quaternion = Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )
    return Pose(
        position=Point(x=x, y=y, z=z),
        orientation=quaternion,
    )


def main():
    import rospy
    from gazebo_msgs.srv import GetWorldProperties, SpawnModel
    from geometry_msgs.msg import Point, Pose, Quaternion

    rospy.init_node("scene_sign_compat")
    world_path = os.path.abspath(
        os.path.expanduser(rospy.get_param("~world_path"))
    )
    mesh_dir = os.path.abspath(
        os.path.expanduser(rospy.get_param("~mesh_dir"))
    )
    with open(world_path, encoding="utf-8") as handle:
        overlays = build_missing_sign_overlays(handle.read())
    if not overlays:
        rospy.loginfo(
            "Scene sign resources resolve through their official file URIs"
        )
        return

    rospy.wait_for_service("/gazebo/get_world_properties")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    world_properties = rospy.ServiceProxy(
        "/gazebo/get_world_properties", GetWorldProperties
    )
    spawn_model = rospy.ServiceProxy(
        "/gazebo/spawn_sdf_model", SpawnModel
    )
    existing = set(world_properties().model_names)
    restored = 0
    for overlay in overlays:
        mesh_path = os.path.join(mesh_dir, overlay.mesh_name)
        if not os.path.isfile(mesh_path):
            rospy.logerr(
                "Cannot restore scene sign %s: package mesh is missing: %s",
                overlay.model_name,
                mesh_path,
            )
            continue
        if overlay.model_name in existing:
            rospy.loginfo(
                "Scene sign compatibility overlay already exists: %s",
                overlay.model_name,
            )
            restored += 1
            continue
        result = spawn_model(
            overlay.model_name,
            overlay.sdf_xml,
            "",
            _pose_from_text(
                overlay.pose_text, Pose, Point, Quaternion
            ),
            "world",
        )
        if not result.success:
            rospy.logerr(
                "Failed to restore scene sign %s: %s",
                overlay.model_name,
                result.status_message,
            )
            continue
        rospy.loginfo(
            "Restored official scene sign %s from model://sign/meshes/%s",
            overlay.model_name,
            overlay.mesh_name,
        )
        restored += 1
    rospy.loginfo(
        "Scene sign compatibility ready: %d/%d overlays active",
        restored,
        len(overlays),
    )


if __name__ == "__main__":
    main()
