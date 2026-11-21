#!/bin/bash
set -euo pipefail
here=$(cd -- "$(dirname "$0")" && pwd)
build=$(realpath "$1")
root=/usr/local/lib/luna-apex-wrist
robot=/etc/systemd/system/apex-robot.service.d/70-luna-ctag.conf
tool=/etc/systemd/system/apex-tool.service.d/70-luna-ctag.conf
for unit in apex-robot apex-tool apex-teleop; do
    [[ $(systemctl show "$unit" -p ActiveState --value) == inactive ]]
done
sha256sum --check "$here/factory.sha256"
[[ -f "$build/libGentoSDK.so" && ! -e "$robot" && ! -e "$tool" && ! -e "$root" ]]
install -d -m 0755 "$root/sdk" "$(dirname "$robot")" "$(dirname "$tool")"
rollback() {
    rm -f -- "$robot" "$tool"
    systemctl daemon-reload
    mv -T -- "$root" "$root.failed-$(date +%s)"
}
trap rollback ERR
install -m 0644 "$build/libGentoSDK.so" "$root/sdk/libGentoSDK.so"
install -m 0755 "$here/launch_robot.sh" "$root/launch_robot.sh"
install -m 0644 "$here/tool_lease.py" "$here/factory.sha256" "$root/"
install -m 0644 "$here/robot-wrist.conf" "$robot"
install -m 0644 "$here/tool-wrist.conf" "$tool"
systemctl daemon-reload
systemctl show apex-robot -p ExecStart --value | grep -Fq "$root/launch_robot.sh"
systemctl show apex-tool -p ExecStart --value | grep -Fq "$root/tool_lease.py"
trap - ERR
