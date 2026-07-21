#!/usr/bin/env bash
# Wait for the gripper board to appear on USB, then bring up gripper + FT and
# PROVE they work (state actually publishing, not just services registered).
#
# Why this exists: when the OpenRB-150 drops off USB, gripper_node either dies
# with "Could not find dynamixel port" or comes up with all services advertised
# while /gripper/state never publishes. The second case is the nasty one -- the
# eval then drives combos into a dead gripper and fails deep inside the
# positioning pass with a confusing AttributeError on s_lift.force.
#
# Usage:  bash scripts/bringup_gripper_ft.sh
# Then just plug the gripper USB back in; this does the rest.

set -uo pipefail
source /opt/ros/humble/setup.bash  2>/dev/null
source ~/ws_ctrl/install/setup.bash 2>/dev/null

echo "=== 1/4  waiting for the gripper board on USB (plug it in now) ==="
for i in $(seq 1 60); do
    if lsusb 2>/dev/null | grep -qiE "robotis|2f5d" && ls /dev/ttyACM* >/dev/null 2>&1; then
        echo "  board detected: $(ls /dev/ttyACM* | tr '\n' ' ')"
        break
    fi
    printf '\r  waiting... %ds' $((i*2)); sleep 2
done
echo
if ! ls /dev/ttyACM* >/dev/null 2>&1; then
    echo "  FAILED: still no /dev/ttyACM* after 2 min."
    echo "  The board is unplugged, unpowered, or dead. Nothing software can do."
    exit 1
fi

echo "=== 2/4  checking the USB link is STABLE (a flapping board is worse than none) ==="
first=$(stat -c '%y' /dev/ttyACM0 2>/dev/null)
for i in $(seq 1 10); do
    sleep 2
    now=$(stat -c '%y' /dev/ttyACM0 2>/dev/null) || { echo "  device VANISHED mid-check"; exit 1; }
    if [ "$now" != "$first" ]; then
        echo "  UNSTABLE: re-enumerated during the check."
        echo "  Reseat/replace the USB cable and check the servo power rail before running the eval."
        exit 1
    fi
done
echo "  stable for 20s"

echo "=== 3/4  starting drivers ==="
pkill -f gripper_node   2>/dev/null; pkill -f ft_sensor_node 2>/dev/null; sleep 3
sg dialout -c "bash -c 'source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash && ros2 run magpie_control gripper_node'" \
    > /tmp/log_gripper.txt 2>&1 &
ros2 run magpie_control ft_sensor_node > /tmp/log_ft.txt 2>&1 &
sleep 8

echo "=== 4/4  verifying (state must actually publish) ==="
rc=0
if timeout 10 ros2 topic echo /gripper/state --once >/dev/null 2>&1; then
    echo "  gripper  OK  (/gripper/state publishing)"
else
    echo "  gripper  DEAD — services may exist but state is not publishing"
    tail -n 5 /tmp/log_gripper.txt; rc=1
fi
if timeout 10 ros2 topic echo /ft_sensor/wrench --once >/dev/null 2>&1; then
    echo "  ft       OK"
else
    echo "  ft       DEAD (box refuses :49152 — it may still be booting; retry in ~60s,"
    echo "           or power-cycle the OptoForce box)"
    tail -n 3 /tmp/log_ft.txt; rc=1
fi

[ $rc -eq 0 ] && echo "
ALL GOOD — restart the notebook kernel, run the setup cells, then b1 (resumes at #38)."
exit $rc
