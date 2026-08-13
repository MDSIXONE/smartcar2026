#!/usr/bin/env bash
set -eo pipefail

source "$HOME/.config/smartcar/ros_network.sh"

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
default_compat_dir="$HOME/.config/smartcar/python_http10_compat"
repo_compat_dir="$script_dir/python_http10_compat"
compat_dir="${SMARTCAR_ROS_HTTP_COMPAT_DIR:-$default_compat_dir}"

if [[ ! -r "$compat_dir/sitecustomize.py" && -r "$repo_compat_dir/sitecustomize.py" ]]; then
    compat_dir="$repo_compat_dir"
fi

if [[ ! -r "$compat_dir/sitecustomize.py" ]]; then
    echo "Cannot find ROS HTTP/1.0 compatibility module: $compat_dir/sitecustomize.py" >&2
    exit 2
fi

export SMARTCAR_ROS_FORCE_HTTP10=1
ros_python_path="$(
    python3 -c \
        'import os, rosgraph; print(os.path.dirname(os.path.dirname(rosgraph.__file__)))'
)"
export PYTHONPATH="$compat_dir:$ros_python_path${PYTHONPATH:+:$PYTHONPATH}"

http_protocol="$(
    python3 -c \
        'import rosgraph.xmlrpc; print(rosgraph.xmlrpc.SilenceableXMLRPCRequestHandler.protocol_version)'
)"
if [[ "$http_protocol" != "HTTP/1.0" ]]; then
    echo "ROS XML-RPC compatibility check failed: protocol is $http_protocol" >&2
    exit 2
fi

if rosparam list >/dev/null 2>&1; then
    echo "ROS Master is already running at $ROS_MASTER_URI"
    exit 0
fi

echo "Starting ROS Master at $ROS_MASTER_URI (ROS_IP=$ROS_IP, XML-RPC=$http_protocol)"
exec roscore
