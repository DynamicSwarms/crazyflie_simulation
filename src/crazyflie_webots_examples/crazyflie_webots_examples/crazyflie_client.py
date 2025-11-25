import rclpy
from rclpy.node import Node
from crazyflie_interfaces.msg import Position, Land, Takeoff, GoTo, GenericLogData

import numpy as np

import threading


class Crazyflie(Node):

    def __init__(self):
        rclpy.init()
        super().__init__("crazyflie_client")

        prefix = "cf2"

        self.cmd_position_pub = self.create_publisher(
            Position, prefix + "/cmd_position", 10
        )
        self.land_pub = self.create_publisher(Land, prefix + "/land", 10)
        self.takeoff_pub = self.create_publisher(Takeoff, prefix + "/takeoff", 10)
        self.goto_pub = self.create_publisher(GoTo, prefix + "/go_to", 10)

        # Spin in a separate thread
        self.thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.thread.start()

    def __del__(self):
        self.thread.join()
        rclpy.shutdown()

    def start(self):
        self._takeoff()

    def stop(self):
        self._land()

    def goto(self, position):
        self._cmd_position(np.array(position).astype(np.float))

    def _takeoff(self):
        msg = Takeoff()
        msg.height = 1.0
        msg.yaw = 0.0
        msg.duration.sec = 3
        self.takeoff_pub.publish(msg)

    def _land(self):
        msg = Land()
        msg.height = 0.0
        msg.yaw = 0.0
        msg.duration.sec = 3
        self.land_pub.publish(msg)

    def _cmd_position(self, position):
        msg = Position()
        msg.x = position[0]
        msg.y = position[1]
        msg.z = position[2]
        msg.yaw = 0.0
        self.cmd_position_pub.publish(msg)

    def _go_to(self, position):
        msg = GoTo()
        msg.relative = False
        msg.goal.x = position[0]
        msg.goal.y = position[1]
        msg.goal.z = position[2]
        msg.yaw = 0.0
        msg.duration.sec = 3
        self.goto_pub.publish(msg)
