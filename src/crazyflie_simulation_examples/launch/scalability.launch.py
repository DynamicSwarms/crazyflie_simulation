from launch import LaunchDescription
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node


def generate_launch_description():
    gateway = Node(
        package="crazyflie_simulation_gateway",
        executable="gateway",
        output="screen",
        sigterm_timeout="10.0",
    )

    crazyflies = Node(
        package="crazyflie_simulation_examples",
        executable="crazyflie_spawner",
        output="screen",
        parameters=[{"count": 50}],
    )

    return LaunchDescription(
        [
            gateway,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=gateway,
                    on_start=[
                        LogInfo(msg="Gateway started, spawning crazyflies..."),
                        TimerAction(period=0.5, actions=[crazyflies]),
                    ],
                )
            ),
        ]
    )
