import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def quaternion_from_matrix_terms():
    # optical = [-y_body, -x_body, -z_body]
    # R(body->optical) =
    # [ 0 -1  0]
    # [-1  0  0]
    # [ 0  0 -1]
    # Corresponding unit quaternion:
    return (math.sqrt(0.5), -math.sqrt(0.5), 0.0, 0.0)


class SimStaticFramesNode(Node):
    def __init__(self):
        super().__init__('sim_static_frames_node')
        self.declare_parameter('body_frame_id', 'base_link')
        self.declare_parameter('camera_link_frame_id', 'down_camera_link')
        self.declare_parameter('camera_optical_frame_id', 'down_camera_optical_frame')
        self.declare_parameter('camera_x_m', 0.0)
        self.declare_parameter('camera_y_m', 0.0)
        self.declare_parameter('camera_z_m', 0.0)

        body = str(self.get_parameter('body_frame_id').value)
        cam_link = str(self.get_parameter('camera_link_frame_id').value)
        cam_opt = str(self.get_parameter('camera_optical_frame_id').value)

        broadcaster = StaticTransformBroadcaster(self)
        stamp = self.get_clock().now().to_msg()
        tf_link = TransformStamped()
        tf_link.header.stamp = stamp
        tf_link.header.frame_id = body
        tf_link.child_frame_id = cam_link
        tf_link.transform.translation.x = float(self.get_parameter('camera_x_m').value)
        tf_link.transform.translation.y = float(self.get_parameter('camera_y_m').value)
        tf_link.transform.translation.z = float(self.get_parameter('camera_z_m').value)
        tf_link.transform.rotation.w = 1.0

        qx, qy, qz, qw = quaternion_from_matrix_terms()
        tf_opt = TransformStamped()
        tf_opt.header.stamp = stamp
        tf_opt.header.frame_id = cam_link
        tf_opt.child_frame_id = cam_opt
        tf_opt.transform.rotation.x = qx
        tf_opt.transform.rotation.y = qy
        tf_opt.transform.rotation.z = qz
        tf_opt.transform.rotation.w = qw
        broadcaster.sendTransform([tf_link, tf_opt])


def main(args=None):
    rclpy.init(args=args)
    node = SimStaticFramesNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
