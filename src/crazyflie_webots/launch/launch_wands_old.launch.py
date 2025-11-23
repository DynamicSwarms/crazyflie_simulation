import os
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.actions.execute_process import ExecuteProcess
import yaml

from typing import List


def create_command(robot_name, description_path):
    webots_controller_path = (
        get_package_share_directory("webots_ros2_driver") + "/scripts/webots-controller"
    )
    webots_ip = EnvironmentVariable("WEBOTS_IP", default_value="127.0.0.1").perform(
        LaunchContext()
    )  # 192.168.56.1

    return (
        webots_controller_path
        + " --robot-name="
        + robot_name
        + " --protocol=tcp --ip-address="
        + str(webots_ip)
        + " --port=1234 ros2 --ros-args -p robot_description:="
        + description_path
    )


def create_wand_controllers(context):
    """
    Create the wand controller nodes.
    """
    package_dir = get_package_share_directory("crazyflie_webots")
    wand_ids_str = LaunchConfiguration("wand_ids").perform(context)
    wand_ids = list(map(int, wand_ids_str.split(",")))

    wand_description_path = os.path.join(package_dir, "resource", "wand.urdf")
    for id in wand_ids:
        name = "Wand0" + str(id) + "_ros_ctrl"
        wand = ExecuteProcess(
            name="Wand",
            cmd=[create_command(name, wand_description_path)],
            output="screen",
            shell=True,
            emulate_tty=True,
            additional_env={"WEBOTS_HOME": get_package_prefix("webots_ros2_driver")},
        )

        yield wand


def generate_launch_description():
    default_wands: str = "1"
    robots_yaml_launch_arg = DeclareLaunchArgument(
        name="wand_ids",
        default_value=default_wands,
        description="List of wand ids to be added (Need to be in Simulation.)",
    )

    return LaunchDescription(
        [robots_yaml_launch_arg, OpaqueFunction(function=create_wand_controllers)]
    )
