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

        # Enable/disable terms
        self.declare_parameter('enable_p', True)
        self.declare_parameter('enable_i', False)
        self.declare_parameter('enable_d', False)

        # Gains
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('ki', 0.5)
        self.declare_parameter('kd', 1.0)

        # Integral term limits (anti-windup via clamping)
        self.declare_parameter('i_min', -1.0)
        self.declare_parameter('i_max', 1.0)

        # Reset-I behavior
        self.declare_parameter('reset_i_on_startup', True)
        self.declare_parameter('reset_i_on_setpoint_change', True)
        self.declare_parameter('setpoint_change_eps', 1e-6)

        # Thrust clamp
        self.declare_parameter('thrust_min', float('-inf'))
        self.declare_parameter('thrust_max', float('inf'))

        self.current_setpoint = 0.0
        self.current_depth = 0.0

        # PID state
        self._i_accum: float = 0.0
        self._prev_depth: Optional[float] = None
        self._prev_time_sec: Optional[float] = None
        self._have_started: bool = False
        self._last_setpoint: Optional[float] = None

        self.thrust_pub = self.create_publisher(msg_type=ActuatorSetpoint,
                                                topic='thrust_setpoint',
                                                qos_profile=1)

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
        new_sp = float(setpoint_msg.data)
        if self.get_parameter('reset_i_on_setpoint_change').value:
            if self._last_setpoint is None or abs(new_sp - self._last_setpoint) > float(
                self.get_parameter('setpoint_change_eps').value
            ):
                self._i_accum = 0.0
        self._last_setpoint = new_sp
        self.current_setpoint = new_sp

    def on_depth(self, depth_msg: DepthStamped):
        # Current measurement and time
        self.current_depth = float(depth_msg.depth)
        t_sec = rclpy.time.Time.from_msg(depth_msg.header.stamp).nanoseconds * 1e-9

        # Logging (throttled)
        self.get_logger().info(
            f"Hi! I'm your controller running. I received a depth of {self.current_depth:.3f} m.",
            throttle_duration_sec=1,
        )

        # Compute control
        thrust = self.compute_control_output(self.current_depth, t_sec)

        # Timestamp choice for output
        timestamp = rclpy.time.Time.from_msg(depth_msg.header.stamp)
        self.publish_vertical_thrust(thrust=thrust, timestamp=timestamp)


    def publish_vertical_thrust(self, thrust: float,
                                timestamp: rclpy.time.Time) -> None:
        msg = ActuatorSetpoint()
        # we want to set the vertical thrust exlusively. mask out xy-components.
        msg.ignore_x = True
        msg.ignore_y = True
        msg.ignore_z = False

        msg.z = thrust

        # Let's add a time stamp
        msg.header.stamp = timestamp.to_msg()

        self.thrust_pub.publish(msg)

    def compute_control_output(self, current_depth: float, t_sec: float) -> float:
        # Read params
        kp = float(self.get_parameter('kp').value)
        ki = float(self.get_parameter('ki').value)
        kd = float(self.get_parameter('kd').value)
        enable_p = bool(self.get_parameter('enable_p').value)
        enable_i = bool(self.get_parameter('enable_i').value)
        enable_d = bool(self.get_parameter('enable_d').value)
        i_min = float(self.get_parameter('i_min').value)
        i_max = float(self.get_parameter('i_max').value)
        thrust_min = float(self.get_parameter('thrust_min').value)
        thrust_max = float(self.get_parameter('thrust_max').value)

        # Reset I on first run if requested
        if not self._have_started:
            if bool(self.get_parameter('reset_i_on_startup').value):
                self._i_accum = 0.0
            self._have_started = True

        # Time step
        dt = 0.0
        if self._prev_time_sec is not None:
            dt = max(0.0, t_sec - self._prev_time_sec)

        # Error (depth setpoint minus measured depth)
        print(self.current_setpoint, current_depth)
        error = float(self.current_setpoint - current_depth)

        # ----- P term -----
        p_term = kp * error if enable_p else 0.0

        # ----- D term -----
        # d(measurement)/dt approximated from last two measurements
        d_term = 0.0
        if enable_d and self._prev_depth is not None and dt > 0.0:
            d_meas = (current_depth - self._prev_depth) / dt
            d_term = kd * d_meas

        # ----- I term with clamping + conditional anti-windup -----
        i_term = 0.0
        if enable_i and dt > 0.0 and ki != 0.0:
            # Conditional integration: stop integrating if integral is at a limit
            # and the error would push it further into saturation.
            allow_integrate = True
            if (self._i_accum >= i_max and error > 0.0) or (self._i_accum <= i_min and error < 0.0):
                allow_integrate = False

            if allow_integrate:
                self._i_accum += error * dt
                # Hard clamp the accumulator
                if self._i_accum > i_max:
                    self._i_accum = i_max
                elif self._i_accum < i_min:
                    self._i_accum = i_min

            i_term = ki * self._i_accum

        # Unsaturated output
        u = p_term + i_term + d_term

        # Optional output saturation (useful in practice)
        if u > thrust_max:
            u_sat = thrust_max
        elif u < thrust_min:
            u_sat = thrust_min
        else:
            u_sat = u

        # Update history
        self._prev_depth = current_depth
        self._prev_time_sec = t_sec

        return float(u_sat)


def main():
    rclpy.init()
    node = DepthControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
