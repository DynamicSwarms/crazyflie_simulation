import rclpy
from rclpy.node import Node
from crazyflie_interfaces.srv import AddCrazyflie
from geometry_msgs.msg import Pose
import yaml


class CrazyflieSpawner(Node):
    def __init__(self):
        super().__init__("crazyflie_spawner")

        self.cli = self.create_client(
            AddCrazyflie, "/crazyflie_simulation_gateway/add_crazyflie"
        )

        self._count = (
            self.declare_parameter("count", 50).get_parameter_value().integer_value
        )

        self._yaml = (
            self.declare_parameter("yaml_path", "").get_parameter_value().string_value
        )

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service...")

    def spawn_all(self):
        flies_to_spawn = []
        if self._yaml:
            with open(self._yaml, "r") as f:
                flies_to_spawn = yaml.safe_load(f)["flies"]
        else:
            for i in range(self._count):
                flies_to_spawn.append(
                    {
                        "id": i,
                        "initial_position": [0.0, 0.0, 0.0],
                        "initial_orientation": [0.0, 0.0, 0.0, 1.0],
                    }
                )

        for fly in flies_to_spawn:
            req = AddCrazyflie.Request()
            req.uri = f"sim://{fly['id']}"
            req.initial_pose = Pose()
            req.initial_pose.position.x = fly["initial_position"][0]
            req.initial_pose.position.y = fly["initial_position"][1]
            req.initial_pose.position.z = fly["initial_position"][2]
            req.initial_pose.orientation.x = fly["initial_orientation"][0]
            req.initial_pose.orientation.y = fly["initial_orientation"][1]
            req.initial_pose.orientation.z = fly["initial_orientation"][2]
            req.initial_pose.orientation.w = fly["initial_orientation"][3]

            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                self.get_logger().debug(f"Spawned {fly['id']}")
            else:
                self.get_logger().error(f"Failed {fly['id']}")


def main():
    rclpy.init()
    node = CrazyflieSpawner()
    node.spawn_all()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
