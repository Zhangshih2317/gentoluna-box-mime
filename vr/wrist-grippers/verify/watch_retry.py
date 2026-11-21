"""Read-only recorder for a supervised Apex gripper retry on Luna."""

import json
import time
from datetime import datetime

import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32, Int16MultiArray, Int32MultiArray


def main():
    rclpy.init()
    node = rclpy.create_node("ctag_retry_observer")
    topics = {"info/vr_connected": Bool, "info/robot_state": Int16MultiArray}
    for side in ("L", "R"):
        topics["control/enable" + side] = Bool
        topics["control/gripperValue" + side] = Float32
        topics["info/wrist485_" + side] = Int32MultiArray
    values, last_seen = {}, {}

    def emit(event):
        print(json.dumps({"time": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                          **event}, separators=(",", ":")), flush=True)

    def receive(topic, message):
        value = message.data
        if hasattr(value, "tolist"):
            value = value.tolist()
        if isinstance(value, float):
            value = round(value, 2)
        if values.get(topic) != value:
            emit({"topic": topic, "value": value})
        values[topic] = value
        last_seen[topic] = time.monotonic()

    for topic, kind in topics.items():
        node.create_subscription(kind, topic, lambda m, t=topic: receive(t, m),
                                 qos_profile_sensor_data)
    try:
        end = time.monotonic() + 1800
        next_summary = time.monotonic()
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            if now >= next_summary:
                emit({"age_seconds": {t: round(now-last_seen[t], 2) if t in last_seen else None
                                      for t in topics}, "last": values})
                next_summary = now + 5
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
