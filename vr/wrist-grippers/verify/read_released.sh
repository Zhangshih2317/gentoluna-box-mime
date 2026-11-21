#!/bin/bash
set -eu
root=$(cd -- "$(dirname -- "$0")/.." && pwd)
exec 8>>/tmp/gento_sdk_6_6_7_190.lock
flock -n 8 || { echo "Controller SDK in use" >&2; exit 75; }
exec 9>>/tmp/luna-gripper-sdk.lock
flock -n 9 || { echo "Wrist transport in use" >&2; exit 75; }
for unit in apex-robot apex-teleop apex-tool; do
    [[ $(systemctl show "$unit" -p ActiveState --value) == inactive ]] ||
        { echo "$unit is not inactive" >&2; exit 75; }
done
ports=$(ss -Huln)
if grep -Eq ':372[0-4][[:space:]]' <<<"$ports"; then
    echo "Controller UDP ports in use" >&2
    exit 75
fi
exec "$root/build/inspect_wrist" --released-controller --preflight-retry
