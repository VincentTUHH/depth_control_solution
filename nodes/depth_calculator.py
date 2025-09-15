#!/usr/bin/env python3
"""
This node takes as input the pressure data and computes a resulting water depth.
"""

import rclpy
from hippo_msgs.msg import DepthStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import FluidPressure


class DepthCalculator(Node):
    def __init__(self):
        super().__init__(node_name='depth_calculator')

        # Parameters
        self.declare_parameter('water_density', 1000.0)     # kg/m^3
        self.declare_parameter('gravity', 9.80665)          # m/s^2
        self.declare_parameter('surface_pressure', 101325.) # Pa
        self.declare_parameter('offset', -0.20)             # m


        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.depth_pub = self.create_publisher(
            msg_type=DepthStamped, topic='depth', qos_profile=1
        )
        self.pressure_sub = self.create_subscription(
            msg_type=FluidPressure,
            topic='pressure',
            callback=self.on_pressure,
            qos_profile=qos,
        )

    def on_pressure(self, pressure_msg: FluidPressure) -> None:
        pressure = pressure_msg.fluid_pressure

        depth = self.pressure_to_depth(pressure=pressure)
        now = self.get_clock().now()
        self.publish_depth_msg(depth=depth, now=now)

    def publish_depth_msg(self, depth: float, now: rclpy.time.Time) -> None:
        msg = DepthStamped()
        msg.header.stamp = now.to_msg()
        msg.depth = depth
        self.depth_pub.publish(msg)

    def pressure_to_depth(self, pressure: float) -> float:
        rho = float(self.get_parameter('water_density').value)
        g = float(self.get_parameter('gravity').value)
        p0 = float(self.get_parameter('surface_pressure').value)
        offset = float(self.get_parameter('offset').value)

        if rho <= 0.0 or g <= 0.0:
            self.get_logger().warn(
                f'Invalid parameters: water_density={rho}, gravity={g}. Returning depth=0.0'
            )
            return 0.0

        depth = -(pressure - p0) / (rho * g) # depth is negative down
        return float(depth + offset)


def main():
    rclpy.init()
    node = DepthCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
