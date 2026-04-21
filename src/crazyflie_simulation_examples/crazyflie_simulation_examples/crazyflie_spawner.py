import rclpy
from rclpy.node import Node
from crazyflie_interfaces.srv import AddCrazyflie


class CrazyflieSpawner(Node):
    def __init__(self):
        super().__init__("crazyflie_spawner")

        self.cli = self.create_client(
            AddCrazyflie, "/crazyflie_simulation_gateway/add_crazyflie"
        )

        self._count = (
            self.declare_parameter("count", 50).get_parameter_value().integer_value
        )

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service...")

    def spawn_all(self):
        for i in range(self._count):
            req = AddCrazyflie.Request()
            req.uri = f"sim://{i}"

            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                self.get_logger().debug(f"Spawned {i}")
            else:
                self.get_logger().error(f"Failed {i}")


def main():
    rclpy.init()
    node = CrazyflieSpawner()
    node.spawn_all()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
