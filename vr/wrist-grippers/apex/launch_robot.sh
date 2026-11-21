#!/bin/bash
set -e
source /etc/apex/apex_ros_env.sh
if [[ ${APEX_ROBOT_PLATFORM:-} != gento || ${APEX_ROBOT_MODEL:-} != luna ]]; then
    exec /bin/bash /etc/apex/apex-launch-robot.sh
fi
if ! sha256sum --status --check /usr/local/lib/luna-apex-wrist/factory.sha256; then
    echo "Factory SDK or Robot changed; CTAG wrist integration disabled for this start." >&2
    exec /bin/bash /etc/apex/apex-launch-robot.sh
fi
export LD_LIBRARY_PATH="/usr/local/lib/luna-apex-wrist/sdk:${LD_LIBRARY_PATH:-}"
exec ros2 launch marvin_ros_control bringup_control_gento_luna.launch.py \
    use_rviz:="${APEX_USE_RVIZ:-false}"
