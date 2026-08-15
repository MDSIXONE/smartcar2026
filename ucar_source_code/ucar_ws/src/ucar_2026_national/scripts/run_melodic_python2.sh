#!/usr/bin/env bash
# Run one ROS Melodic Python node with a Python-2-only module search path.
#
# The vehicle also has a Python 3 cv_bridge overlay.  Old Catkin artifacts can
# otherwise make a Python 2 process load a Python 3 tf2 extension and fail with
# "dynamic module does not define init function (init_tf2)".
set -eo pipefail

python2_paths=(
  "$HOME/ucar_ws/devel/lib/python2.7/dist-packages"
  "/opt/ros/melodic/lib/python2.7/dist-packages"
)

IFS=':' read -r -a inherited_python_paths <<<"${PYTHONPATH:-}"
for path_entry in "${inherited_python_paths[@]}"; do
  [[ -n "$path_entry" ]] || continue
  case "$path_entry" in
    */python3|*/python3/*|*/python3.*|*/python3.*/*)
      continue
      ;;
  esac
  python2_paths+=("$path_entry")
done

clean_pythonpath=""
for path_entry in "${python2_paths[@]}"; do
  case ":${clean_pythonpath}:" in
    *":${path_entry}:"*)
      continue
      ;;
  esac
  if [[ -z "$clean_pythonpath" ]]; then
    clean_pythonpath="$path_entry"
  else
    clean_pythonpath="${clean_pythonpath}:$path_entry"
  fi
done

export PYTHONPATH="$clean_pythonpath"
exec /usr/bin/python2 "$@"
