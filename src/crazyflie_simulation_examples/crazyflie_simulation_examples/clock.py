import rclpy
from rclpy.node import Node
from rclpy.time import Time

from rosgraph_msgs.msg import Clock


class ClockNode(Node):
    def __init__(self):
        super().__init__("clock_node")
        self.pub = self.create_publisher(Clock, "/clock", 10)
        self.rate = (
            self.declare_parameter("rate", 2.0).get_parameter_value().double_value
        )

        # Use wall time as reference so this works even when use_sim_time:=true
        self.wall_start_ns = self.get_clock().now().nanoseconds
        self.sim_start_ns = 0  # start sim time at 0

        self.timer = self.create_timer(0.01, self.tick)

    def tick(self):
        wall_now_ns = self.get_clock().now().nanoseconds
        wall_elapsed_ns = wall_now_ns - self.wall_start_ns

        sim_now_ns = int(self.sim_start_ns + self.rate * wall_elapsed_ns)

        msg = Clock()
        msg.clock = Time(nanoseconds=sim_now_ns).to_msg()
        self.pub.publish(msg)


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
