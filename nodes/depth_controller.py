#!/usr/bin/env python3
"""
This node is your depth controller.
It takes as input a current depth and a given depth setpoint.
Its output is a thrust command to the BlueROV's actuators.
"""

import rclpy
from hippo_control_msgs.msg import ActuatorSetpoint
from hippo_msgs.msg import DepthStamped, Float64Stamped
from rclpy.node import Node


class DepthControlNode(Node):
    def __init__(self):
        super().__init__(node_name='depth_controller')

        self.current_setpoint = 0.0
        self.current_depth = 0.0

        self.thrust_pub = self.create_publisher(
            msg_type=ActuatorSetpoint, topic='thrust_setpoint', qos_profile=1
        )

        self.setpoint_sub = self.create_subscription(
            msg_type=Float64Stamped,
            topic='depth_setpoint',
            callback=self.on_setpoint,
            qos_profile=1,
        )
        self.depth_sub = self.create_subscription(
            msg_type=DepthStamped,
            topic='depth',
            callback=self.on_depth,
            qos_profile=1,
        )

    def on_setpoint(self, setpoint_msg: Float64Stamped):
        # We received a new setpoint! Let's save it, so that we can use it as
        # soon as we receive new depth data.
        self.current_setpoint = setpoint_msg.data

    def on_depth(self, depth_msg: DepthStamped):
        # We received a new depth message! Now we can get to action!
        current_depth = depth_msg.depth

        self.get_logger().info(
            f"Hi! I'm your controller running. "
            f'I received a depth of {current_depth} m.',
            throttle_duration_sec=1,
        )

        thrust = self.compute_control_output(current_depth)
        # either set the timestamp to the current time or set it to the
        # stamp of `depth_msg` because the control output corresponds to this
        # point in time. Both choices are meaningful.
        # option 1:
        # timestamp = self.get_clock().now()
        # option 2:
        timestamp = rclpy.time.Time.from_msg(depth_msg.header.stamp)
        self.publish_vertical_thrust(thrust=thrust, timestamp=timestamp)

    def publish_vertical_thrust(
        self, thrust: float, timestamp: rclpy.time.Time
    ) -> None:
        msg = ActuatorSetpoint()
        # we want to set the vertical thrust exlusively. mask out xy-components.
        msg.ignore_x = True
        msg.ignore_y = True
        msg.ignore_z = False

        msg.z = thrust

        # Let's add a time stamp
        msg.header.stamp = timestamp.to_msg()

        self.thrust_pub.publish(msg)

    def compute_control_output(self, current_depth: float) -> float:
        # TODO: Apply the PID control
        thrust_z = 0.0  # This doesn't seem right yet...
        return thrust_z


def main():
    rclpy.init()
    node = DepthControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
