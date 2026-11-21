from pathlib import Path

from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(__file__).with_name("apex-color60.yaml"))
    device = "/dev/v4l/by-path/platform-14160000.pcie-pci-0004:01:00.0-usb-0:2:1.3-video-index0"
    # The native Apex V4L2 source negotiates 30 fps even for a 60 fps output.
    capture = ExecuteProcess(
        cmd=[
            "/usr/bin/gst-launch-1.0", "-q", "v4l2src", f"device={device}",
            "do-timestamp=true", "!",
            "video/x-raw,format=YUY2,width=960,height=540,framerate=60/1", "!",
            "queue", "leaky=downstream", "max-size-buffers=2",
            "max-size-bytes=0", "max-size-time=0", "!",
            "nvvidconv", "!", "video/x-raw(memory:NVMM),format=NV12", "!",
            "nvv4l2h264enc", "bitrate=8000000", "insert-sps-pps=true",
            "iframeinterval=30", "idrinterval=30", "num-B-Frames=0", "!",
            "rtph264pay", "pt=96", "mtu=1200", "config-interval=1", "!",
            "udpsink", "host=127.0.0.1", "port=5600", "sync=false", "async=false",
        ],
        output="screen",
    )
    signal = Node(
        package="gmsl_quadcam", executable="webrtc_signal_server.py",
        name="webrtc_signal_server", output="screen",
        parameters=[{"host": "0.0.0.0", "port": 8554}],
    )
    video = Node(
        package="gmsl_quadcam", executable="quad_csi_quickview",
        name="quad_csi_quickview", output="screen",
        parameters=[config, {"camera_config": config}],
    )
    processes = [capture, signal, video]
    handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason="Color streaming process exited"))],
        ))
        for process in processes
    ]
    return LaunchDescription(handlers + processes)
