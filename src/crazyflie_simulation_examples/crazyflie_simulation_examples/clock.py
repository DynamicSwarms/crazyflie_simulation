import rclpy
from rclpy.node import Node, SetParametersResult
from rclpy.time import Time

from rosgraph_msgs.msg import Clock


class ClockNode(Node):
    def __init__(self):
        super().__init__("clock_node")
        self.pub = self.create_publisher(Clock, "/clock", 10)
        self.rate = (
            self.declare_parameter("rate", 2.0).get_parameter_value().double_value
        )

        self.param_callback_handle = self.add_on_set_parameters_callback(
            self.on_set_parameters
        )

        self.ros_ns = 0

        self.timer = self.create_timer(0.1 / self.rate, self.tick)
        # We aim for a 10Hz clock in the given rate

    def tick(self):
        self.ros_ns += 0.1 * 1e9

        msg = Clock()
        msg.clock = Time(nanoseconds=self.ros_ns).to_msg()
        self.pub.publish(msg)

    def on_set_parameters(self, params):
        for param in params:
            if param.name == "rate":
                self.rate = param.get_parameter_value().double_value
                self.timer.cancel()
                self.timer = self.create_timer(0.1 / self.rate, self.tick)
                self.get_logger().info(f"Set rate to {self.rate}")

        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    node = ClockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
