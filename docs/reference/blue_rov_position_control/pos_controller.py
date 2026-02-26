#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, Point, Quaternion
from mavros_msgs.msg import OverrideRCIn
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from scipy.spatial.transform import Rotation as R
import numpy as np


YAW_CH = 4
FORWARD_CH = 5
LATERAL_CH = 6
PWM_MID = 1500
PWM_MIN = 0
PWM_MAX = 2000


class PIDController:
    def __init__(self, p_gain: float = 0.0, i_gain: float = 0.0, d_gain: float = 0.0):
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.last_error = 0.0
        self.error_sum = 0.0

    def set_parameters(self, p_gain: float = None, i_gain: float = None, d_gain: float = None):
        if p_gain is not None:
            self.p_gain = p_gain
        if i_gain is not None:
            self.i_gain = i_gain
        if d_gain is not None:
            self.d_gain = d_gain
        self.last_error = 0.0
        self.error_sum = 0.0

    def set_p_gain(self, p_gain: float):
        self.p_gain = p_gain

    def set_i_gain(self, i_gain: float):
        self.i_gain = i_gain
        self.error_sum = 0.0

    def set_d_gain(self, d_gain: float):
        self.d_gain = d_gain

    def reset_integrator(self):
        self.error_sum = 0.0

    def compute_control_output(self, error: float, dt: float) -> float:
        self.error_sum += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        output = (self.p_gain * error) + (self.i_gain * self.error_sum) + (self.d_gain * derivative)
        self.last_error = error
        return output
    

class PositionControlNode(Node):
    def __init__(self):
        super().__init__(node_name='position_controller')

        self.setpoint_x = 0.0
        self.setpoint_y = 0.0
        self.setpoint_r = 0.0

        self.pid_x = PIDController(p_gain=0.5, i_gain=0.0, d_gain=0.0)
        self.pid_y = PIDController(p_gain=0.5, i_gain=0.0, d_gain=0.0)
        self.pid_r = PIDController(p_gain=0.5, i_gain=0.0, d_gain=0.0)

        self.msg = OverrideRCIn()

        # ROS paramter for proportional gain x-axis
        self.declare_parameter(
            name='p_gain_x', 
            value=0.0, 
            descriptor=ParameterDescriptor(description='Proportional control gain x-axis.')
        )
        # ROS parameter for integral gain x-axis
        self.declare_parameter(
            name='i_gain_x', 
            value=0.0, 
            descriptor=ParameterDescriptor(description='Integral control gain x-axis.')
        )
        # ROS parameter for differential gain x-axis
        self.declare_parameter(
            name='d_gain_x', 
            value=0.0, 
            descriptor=ParameterDescriptor(description='Differential control gain x-axis.')
        )
        # ROS paramter for proportional gain y-axis
        self.declare_parameter(
            name='p_gain_y', 
            value=0.0, 
            descriptor=ParameterDescriptor(description='Proportional control gain y-axis.')
        )
        # ROS parameter for integral gain y-axis
        self.declare_parameter(
            name='i_gain_y', 
            value=0.0, 
            descriptor=ParameterDescriptor(description='Integral control gain y-axis.')
        )
        # ROS parameter for differential gain y-axis
        self.declare_parameter(
            name='d_gain_y', 
            value=0.0, 
            descriptor=ParameterDescriptor(description='Differential control gain y-axis.')
        )
        # ROS paramter for proportional gain yaw-axis
        self.declare_parameter(
            name='p_gain_r', 
            value=0.0, 
            descriptor=ParameterDescriptor(description='Proportional control gain yaw-axis.')
        )
        # ROS parameter for integral gain yaw-axis
        self.declare_parameter(
            name='i_gain_r', 
            value=0.0, 
            descriptor=ParameterDescriptor(description='Integral control gain yaw-axis.')
        )
        # ROS parameter for differential gain yaw-axis
        self.declare_parameter(
            name='d_gain_r', 
            value=0.0, 
            descriptor=ParameterDescriptor(description='Differential control gain yaw-axis.')
        )
        self.declare_parameter(
            name='setpoint_x',
            value=0.0,
            descriptor=ParameterDescriptor(description='Setpoint for x-axis position.')
        )
        self.declare_parameter(
            name='setpoint_y',
            value=0.0,
            descriptor=ParameterDescriptor(description='Setpoint for y-axis position.')
        )
        self.declare_parameter(
            name='setpoint_r',
            value=0.0,
            descriptor=ParameterDescriptor(description='Setpoint for yaw-axis position.')
        )

        self.add_on_set_parameters_callback(self.parameters_callback)

        # ROS publisher for actuators
        self.thrust_pub = self.create_publisher(
            msg_type=OverrideRCIn, topic='/mavros/rc/override', qos_profile=1
        )

        # ROS publisher for error
        self.error_pub = self.create_publisher(
            msg_type=Vector3Stamped, topic='/BlueROV2/error', qos_profile=1
        )

        # ROS subscriber for pose
        self.pose_sub = self.create_subscription(
            msg_type=PoseStamped,
            topic='/BlueROV2/pose',
            callback=self.on_pose,
            qos_profile=1,
        )

        self.timer = self.create_timer(
            timer_period_sec=1 / 20,
            callback=self.on_timer,
        )


    def parameters_callback(self, params):
        success = False

        for param in params:
            if param.name == "p_gain_x":
                if param.type_ == Parameter.Type.DOUBLE:
                    self.pid_x.set_p_gain(param.value)
                    success = True
            if param.name == "i_gain_x":
                if param.type_ == Parameter.Type.DOUBLE:
                    self.pid_x.set_i_gain(param.value)
                    success = True
            if param.name == "d_gain_x":
                if param.type_ == Parameter.Type.DOUBLE:
                    self.pid_x.set_d_gain(param.value)
                    success = True
            if param.name == "p_gain_y":
                if param.type_ == Parameter.Type.DOUBLE:
                    self.pid_y.set_p_gain(param.value)
                    success = True
            if param.name == "i_gain_y":
                if param.type_ == Parameter.Type.DOUBLE:
                    self.pid_y.set_i_gain(param.value)
                    success = True
            if param.name == "d_gain_y":
                if param.type_ == Parameter.Type.DOUBLE:
                    self.pid_y.set_d_gain(param.value)
                    success = True
            if param.name == "p_gain_r":
                if param.type_ == Parameter.Type.DOUBLE:
                    self.pid_r.set_p_gain(param.value)
                    success = True
            if param.name == "i_gain_r":
                if param.type_ == Parameter.Type.DOUBLE:
                    self.pid_r.set_i_gain(param.value)
                    success = True
            if param.name == "d_gain_r":
                if param.type_ == Parameter.Type.DOUBLE:
                    self.pid_r.set_d_gain(param.value)
                    success = True
            if param.name == "setpoint_x":
                if param.type_ == Parameter.Type.DOUBLE:
                    self.setpoint_x = param.value
                    success = True
            if param.name == "setpoint_y":
                if param.type_ == Parameter.Type.DOUBLE:
                    self.setpoint_y = param.value
                    success = True
            if param.name == "setpoint_r":
                if param.type_ == Parameter.Type.DOUBLE:
                    self.setpoint_r = param.value
                    success = True
            

        self.get_logger().info(
            f"Set controller parameters.\n"
            f"X-Axis:  p-gain={self.pid_x.p_gain}  i-gain={self.pid_x.i_gain}  d-gain={self.pid_x.d_gain}\n"
            f"Y-Axis:  p-gain={self.pid_y.p_gain}  i-gain={self.pid_y.i_gain}  d-gain={self.pid_y.d_gain}\n"
            f"R-Axis:  p-gain={self.pid_r.p_gain}  i-gain={self.pid_r.i_gain}  d-gain={self.pid_r.d_gain}\n"
            f"Setpoints: x={self.setpoint_x:6.2f}, y={self.setpoint_y:6.2f}, r={self.setpoint_r:6.2f}"
        )

        return SetParametersResult(successful=success)


    def on_pose(self, pose_msg: PoseStamped):
        position = pose_msg.pose.position
        quaternion = pose_msg.pose.orientation
        # self.get_logger().info(
        #     f'\nReceived a position of x={position.x:6.2f}, y={position.y:6.2f}\n'
        #     f'position setpoint at x={self.setpoint_x:6.2f}, y={self.setpoint_y:6.2f}',
        #     throttle_duration_sec=1,
        # )

        [error_x, error_y, error_r] = self.compute_control_error(position, quaternion)
        [thrust_x, thrust_y, thrust_r] = self.compute_control_output(error_x, error_y, error_r)

        self.get_logger().info(
            f'\nThrust command: x={thrust_x:6.2f}, y={thrust_y:6.2f}, r={thrust_r:6.2f}',
            throttle_duration_sec=1,
        )
        self.publish_thrust(thrust_fwrd=thrust_y, thrust_lat=thrust_x, thrust_yaw=thrust_r)


    def on_timer(self):
        #self.thrust_pub.publish(self.msg)
        pass


    def publish_thrust(
            self, thrust_fwrd: float, thrust_lat: float, thrust_yaw: float
        ) -> None:
        self.msg = OverrideRCIn()
        # we want to set the horizontal thrust exlusively.
        self.msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * len(self.msg.channels)
        self.msg.channels[YAW_CH - 1] = PWM_MID + thrust_yaw
        self.msg.channels[FORWARD_CH - 1] = PWM_MID + thrust_fwrd
        self.msg.channels[LATERAL_CH - 1] = PWM_MID + thrust_lat

        self.thrust_pub.publish(self.msg)


    def compute_control_error(self, position: Point, quaternion: Quaternion) -> list[float]:
        r = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        euler = r.as_euler('zyx', degrees=True)
        r_inv = r.inv()
        target = np.array([position.x, position.y, position.z])
        to_target = r_inv.apply(target)
        
        self.get_logger().info(
            f'\nCurrent orientation: roll={euler[2]:6.2f}, pitch={euler[1]:6.2f}, yaw={euler[0]:6.2f}',
            throttle_duration_sec=1,
        )
        # self.get_logger().info(
        #     f'\nError to target: x={to_target[0]:6.2f}, y={to_target[1]:6.2f}, z={to_target[2]:6.2f}',
        #     throttle_duration_sec=1,
        # )

        error_x = self.setpoint_x - to_target[0]
        error_y = self.setpoint_y - to_target[1]
        error_r = self.setpoint_r + euler[0]

        error_msg = Vector3Stamped()
        error_msg.header.stamp = self.get_clock().now().to_msg()
        error_msg.header.frame_id = 'BlueROV2'
        error_msg.vector.x = error_x
        error_msg.vector.y = error_y
        error_msg.vector.z = error_r
        self.error_pub.publish(error_msg)
        
        return [error_x, error_y, error_r]
    

    def compute_control_output(
            self, error_fwrd: float, error_lat: float, error_yaw: float
        ) -> list[float]:

        thrust_x = self.pid_x.compute_control_output(error_fwrd, self.get_clock().now().nanoseconds * 1e-9)
        thrust_y = self.pid_y.compute_control_output(error_lat, self.get_clock().now().nanoseconds * 1e-9)
        thrust_r = self.pid_r.compute_control_output(error_yaw, self.get_clock().now().nanoseconds * 1e-9)

        return [thrust_x, thrust_y, thrust_r]


def main():
    rclpy.init()
    node = PositionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()