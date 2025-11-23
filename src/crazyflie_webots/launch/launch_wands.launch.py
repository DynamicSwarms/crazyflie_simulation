import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def create_wand_controllers(context):
    """
    Create the wand controller nodes.
    """
    wand_ids_str = LaunchConfiguration("wand_ids").perform(context)
    wand_ids = list(map(int, wand_ids_str.split(",")))

    webots_port = int(LaunchConfiguration("webots_port").perform(context))
    webots_use_tcp = LaunchConfiguration("webots_use_tcp").perform(context).lower() in ['true', '1', 'yes', 'on']
    webots_tcp_ip = LaunchConfiguration("webots_tcp_ip").perform(context)

    for id in wand_ids:
        wand = Node(
            package="crazyflie_webots_cpp",
            executable="wand",
            name="Wand0" + str(id),
            parameters=[
                {"id": id,
                 "webots_port": webots_port,
                 "webots_use_tcp": webots_use_tcp,
                 "webots_tcp_ip": webots_tcp_ip}
            ],
            output="screen")    
        yield wand


def generate_launch_description():
    default_wands: str = "1"
    robots_yaml_launch_arg = DeclareLaunchArgument(
        name="wand_ids",
        default_value=default_wands,
        description="List of wand ids to be added (Need to be in Simulation.)",
    )

    webots_port_launch_arg = DeclareLaunchArgument(
        name="webots_port",
        default_value="1234",
        description="Webots port.")

    webots_use_tcp_launch_arg = DeclareLaunchArgument(
        name="webots_use_tcp",
        default_value="false",
        description="Use TCP connection to Webots (else ipc).")
    
    webots_tcp_ip_launch_arg = DeclareLaunchArgument(
        name="webots_tcp_ip",
        default_value="127.0.0.1",
        description="Webots TCP IP address.")
    

    return LaunchDescription(
        [robots_yaml_launch_arg, webots_port_launch_arg, webots_use_tcp_launch_arg, webots_tcp_ip_launch_arg, OpaqueFunction(function=create_wand_controllers)]
    )
