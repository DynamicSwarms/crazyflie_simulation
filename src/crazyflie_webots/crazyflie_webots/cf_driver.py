from crazyflie_webots.wb_ros_driver import WebotsRosDriver

from crazyflie_driver.high_level_commander import WebotsHighLevelCommander
from crazyflie_driver.generic_commander import WebotsGenericCommander
from crazyflie_driver.rpyt_commander import WebotsRPYTCommander
from crazyflie_driver.logging import WebotsLogging
from crazyflie_driver.parameters import WebotsParameters

from crazyflie_interfaces_python.server.logblock import LogBlockServer

from geometry_msgs.msg import PoseArray, PoseStamped
from tf_transformations import quaternion_about_axis
from crazyflie_interfaces.msg import PoseStampedArray
from crazyflie_interfaces.msg import GenericLogData
from typing import List
from lifecycle_msgs.msg import TransitionEvent, State


class CrazyflieDriverNode(WebotsRosDriver):
    def init(self, webots_node, properties):
        super().init(webots_node, properties, tf_publishing=False)
        self.target_field = self.wb_node.getField("target")
        self.range_finder = self.wb_node.getField("zrange")
        self.vbat_field = self.wb_node.getField("vbat")
        self.charge_current_field = self.wb_node.getField("chargeCurrent")
        self.pm_state_field = self.wb_node.getField("pm_state")

        hl_commander = WebotsHighLevelCommander(
            self.ros_node, self.set_target, self.get_position
        )

        generic_commander = WebotsGenericCommander(
            self.ros_node, self.set_target, self.get_position
        )

        rpyt_commander = WebotsRPYTCommander(self.ros_node)

        parameters = WebotsParameters(self.ros_node)

        logging_variables = {
            "range.zrange": self.get_zrange,
            "stateEstimate.x": lambda: self.get_position()[0],
            "stateEstimate.y": lambda: self.get_position()[1],
            "stateEstimate.z": lambda: self.get_position()[2],
        }
        if self.vbat_field is not None:
            logging_variables["pm.vbat"] = self.get_vbat
        if self.charge_current_field is not None:
            logging_variables["pm.chargeCurrent"] = self.get_charge_current
        if self.pm_state_field is not None:
            logging_variables["pm.state"] = self.get_pm_state

        logging = WebotsLogging(self.ros_node, logging_variables)

        # block = LogBlockServer(self.ros_node, "zrange")
        # logging.create_log_block(["range.zrange"], block)
        # block._log_block_start_callback(200)
        self.pose_publisher = self.ros_node.create_publisher(
            msg_type=PoseStampedArray, topic="cf_positions", qos_profile=1
        )

        self.battery_publisher = self.ros_node.create_publisher(
            msg_type=GenericLogData, topic="~/state", qos_profile=1
        )

        self.ros_node.create_timer(1 / 10.0, self.publish_pose)

        self.transition_event_publisher = self.ros_node.create_publisher(
            msg_type=TransitionEvent,
            topic=self.getName() + "/transition_event",
            qos_profile=10,
        )

        def publish_transition_event(goal_state_id):
            event = TransitionEvent()
            event.goal_state.id = goal_state_id
            event.goal_state.label = (
                "inactive" if goal_state_id == State.PRIMARY_STATE_INACTIVE else ""
            )
            self.transition_event_publisher.publish(event)

        # Publish transition event to 'inactive' state after initialization
        publish_transition_event(State.PRIMARY_STATE_INACTIVE)

    def publish_pose(self):
        state = GenericLogData()
        state.values.append(4.2)  # Always full battery
        self.battery_publisher.publish(state)

        pos = self.get_position()
        rot = self.get_rotation()
        quat = quaternion_about_axis(rot[3], rot[:3])

        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pos
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = quat
        pose.header.frame_id = self.getName()
        pose.header.stamp = self.ros_node.get_clock().now().to_msg()

        msg = PoseStampedArray()
        msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.poses = [pose]
        self.pose_publisher.publish(msg)

    def get_zrange(self) -> float:
        return self.range_finder.getSFFloat()

    def set_target(self, target: List[float]) -> None:
        self.target_field.setSFVec3f(target)

    def get_vbat(self) -> float:
        if self.vbat_field is not None:
            return self.vbat_field.getSFFloat()
        return 0.0

    def get_charge_current(self) -> float:
        if self.charge_current_field is not None:
            return self.charge_current_field.getSFFloat()
        return 0.0

    def get_pm_state(self) -> float:
        if self.pm_state_field is not None:
            return float(self.pm_state_field.getSFInt32())
        return 0.0

    def step(self):
        super().step()
