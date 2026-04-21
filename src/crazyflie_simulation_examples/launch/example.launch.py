from launch import LaunchDescription
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node


def generate_launch_description():
    gateway = Node(
        package="crazyflie_simulation_gateway",
        executable="gateway",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    spawn_crazyflie = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " service call ",
                "/crazyflie_simulation_gateway/add_crazyflie  ",
                "crazyflie_interfaces/srv/AddCrazyflie ",
                '"{uri: "sim://0"}"',
            ]
        ],
        shell=True,
    )

    move_crazyflie = Node(
        package="crazyflie_simulation_examples",
        executable="crazyflie",
        output="screen",
    )

    return LaunchDescription(
        [
            gateway,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=gateway,
                    on_start=[
                        LogInfo(msg="Gateway started, spawning crazyflie..."),
                        spawn_crazyflie,
                    ],
                )
            ),
            move_crazyflie,
        ]
    )
