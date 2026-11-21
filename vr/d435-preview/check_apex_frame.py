import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

rclpy.init()
node = Node("d435_apex_image_check")
received = []


def on_image(message):
    received.append((time.monotonic(), len(message.data)))
    if len(received) == 1:
        Path("/home/nvidia/d435-preview/apex-frame.jpg").write_bytes(bytes(message.data))


subscription = node.create_subscription(
    CompressedImage, "/quad_tile/compressed", on_image, qos_profile_sensor_data)
deadline = time.monotonic() + 12
while time.monotonic() < deadline and len(received) < 30:
    rclpy.spin_once(node, timeout_sec=.3)
node.destroy_node()
rclpy.shutdown()
print(json.dumps({
    "frames": len(received),
    "first_frame_bytes": received[0][1] if received else 0,
    "measured_fps": (len(received) - 1) / (received[-1][0] - received[0][0])
    if len(received) > 1 else 0,
}))
raise SystemExit(0 if len(received) >= 10 else 1)
