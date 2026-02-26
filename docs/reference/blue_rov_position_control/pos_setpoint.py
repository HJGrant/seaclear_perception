#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from geographic_msgs.msg import GeoPoseStamped


class PosSetpointNode(Node):
    def __init__(self):
        super().__init__(node_name='pos_setpoint_publisher')

        self.start_time = self.get_clock().now()

        self.setpoint_upper = -0.6  # in m
        self.setpoint_lower = -0.8  # in m
        self.duration = 60.0  # full cycle duration in seconds
        self.ratio = 0.7 # duration ratio of lower setpoint
        self.ramp = 2.0 # in seconds

        self.declare_parameter(
            name='setpoint_upper', 
            value=-0.65, 
            descriptor=ParameterDescriptor(description='Upper Setpoint.')
        )
        self.declare_parameter(
            name='setpoint_lower', 
            value=-0.8, 
            descriptor=ParameterDescriptor(description='Lower Setpoint.')
        )
        self.declare_parameter(
            name='duration', 
            value=60.0, 
            descriptor=ParameterDescriptor(description='Duration.')
        )
        self.declare_parameter(
            name='ratio',
            value=0.7,
            descriptor=ParameterDescriptor(description='Ratio')
        )
        self.declare_parameter(
            name='ramp', 
            value=5.0, 
            descriptor=ParameterDescriptor(description='Ramp.')
        )

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.depth_setpoint_pub = self.create_publisher(
            msg_type=GeoPoseStamped,
            topic='/mavros/setpoint_position/global',
            qos_profile=1,
        )

        self.timer = self.create_timer(
            timer_period_sec=1 / 20,
            callback=self.on_timer,
        )


    def parameters_callback(self, params):
        success = False

        for param in params:
            if param.name == "setpoint_upper":
                if param.type_ == Parameter.Type.DOUBLE and param.value < 0.0:
                    if param.value < self.setpoint_lower:
                        self.get_logger().warn(
                            f"Upper setpoint {param.value} is lower than lower setpoint {self.setpoint_lower}. "
                            "Not changing upper setpoint!"
                        )
                    else:
                        self.setpoint_upper = param.value
                        success = True
            if param.name == "setpoint_lower":
                if param.type_ == Parameter.Type.DOUBLE and param.value < 0.0:
                    if param.value > self.setpoint_upper:
                        self.get_logger().warn(
                            f"Lower setpoint {param.value} is higher than upper setpoint {self.setpoint_upper}. "
                            "Not changing lower setpoint!"
                        )
                    else:
                        self.setpoint_lower = param.value
                        success = True
            if param.name == "duration":
                if param.type_ == Parameter.Type.DOUBLE and param.value > 0.0:
                    self.duration = param.value
                    success = True
            if param.name == "ratio":
                if param.type_ == Parameter.Type.DOUBLE and param.value > 0.0 and param.value < 1.0:
                    self.ratio = param.value
                    success = True
            if param.name == "ramp":
                if param.type_ == Parameter.Type.DOUBLE and param.value > 0.0:
                    self.ramp = param.value
                    success = True

        self.get_logger().info(
            f"Set setpoint parameters.\n"
            f"Upper Setpoint={self.setpoint_upper}\n"
            f"Lower Setpoint={self.setpoint_lower}\n"
            f"Duration={self.duration}\n"
            f"Ratio={self.ratio}\n"
            f"Ramp={self.ramp}\n"
        )

        return SetParametersResult(successful=success)


    def on_timer(self) -> None:
        # change this for other setpoint functions
        now = self.get_clock().now()
        time = self.start_time - now
        i = time.nanoseconds * 1e-9 % self.duration

        if i > (self.ratio * self.duration):
            setpoint = min(self.setpoint_upper, self.setpoint_lower + (self.setpoint_upper - self.setpoint_lower) * ((i - self.ratio * self.duration) / self.ramp))
        else:
            setpoint = max(self.setpoint_lower, self.setpoint_upper + (self.setpoint_lower - self.setpoint_upper) * (i / self.ramp))

        self.publish_setpoint(setpoint=setpoint)
        self.get_logger().info(f"Setpoint={setpoint}\n", throttle_duration_sec=1)


    def publish_setpoint(self, setpoint: float) -> None:
        msg = GeoPoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.altitude = setpoint

        self.depth_setpoint_pub.publish(msg)


def main():
    rclpy.init()
    node = PosSetpointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()