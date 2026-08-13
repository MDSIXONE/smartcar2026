"""Force ROS 1 XML-RPC servers to use HTTP/1.0 when explicitly enabled.

The UCar runs a Linux 4.9 kernel.  ROS itself disables HTTP/1.1 XML-RPC
servers on Linux kernels older than 4.16, but the Noetic Master runs inside a
newer WSL kernel and cannot infer the remote vehicle kernel.  In the current
WSL mirrored-network path, HTTP/1.1 Master replies intermittently remain
unacknowledged and block roslaunch system.multicall().

Python imports ``sitecustomize`` automatically at interpreter startup.  The
launcher adds this directory to PYTHONPATH only for the ROS Master process tree
and sets the opt-in environment variable below.
"""

import os


if os.environ.get("SMARTCAR_ROS_FORCE_HTTP10") == "1":
    try:
        import rosgraph.xmlrpc
    except ImportError:
        pass
    else:
        rosgraph.xmlrpc.SilenceableXMLRPCRequestHandler.protocol_version = (
            "HTTP/1.0"
        )
