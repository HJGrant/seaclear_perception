import math
from typing import Tuple

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def clamp(value: float, limit: float) -> float:
    if limit <= 0.0:
        return value
    return max(-limit, min(limit, value))


def world_from_body(vx: float, vy: float, yaw: float) -> Tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (vx * c - vy * s, vx * s + vy * c)


def make_odom(frame_id: str, child_frame_id: str, stamp, x: float, y: float, z: float,
              yaw: float, vx: float, vy: float, vz: float, wyaw: float) -> Odometry:
    msg = Odometry()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.position.z = float(z)
    msg.pose.pose.orientation = yaw_to_quaternion(yaw)
    msg.twist.twist.linear.x = float(vx)
    msg.twist.twist.linear.y = float(vy)
    msg.twist.twist.linear.z = float(vz)
    msg.twist.twist.angular.z = float(wyaw)
    return msg
