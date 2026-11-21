from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    root = "/home/nvidia/d435-preview"
    bridge = ExecuteProcess(
        cmd=["/home/nvidia/miniconda3/envs/robot/bin/python",
             root + "/rgbd_bridge.py", "--serial", "348122070928"],
        additional_env={"PYTHONPATH": ""},
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
        parameters=[root + "/apex-rgbd.yaml",
                    {"camera_config": root + "/apex-rgbd.yaml"}],
    )
    # Stop the whole camera launch if an essential process exits.
    handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason="RGB-D camera process exited"))],
        ))
        for process in (bridge, signal, video)
    ]
    return LaunchDescription(handlers + [bridge, signal, video])
