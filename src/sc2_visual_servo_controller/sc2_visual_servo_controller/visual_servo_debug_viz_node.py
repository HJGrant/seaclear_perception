from typing import Optional

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from sc2_visual_servo_interfaces.msg import TargetObservation
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class VisualServoDebugVizNode(Node):
    def __init__(self):
        super().__init__('visual_servo_debug_viz_node')

        self.declare_parameter('target_topic', '/sc2/visual_servo/target_obs')
        self.declare_parameter('cmd_topic', '/sc2/visual_servo/cmd_vel_body')
        self.declare_parameter('grasp_pose_topic', '/sc2/visual_servo/grasp_pose')
        self.declare_parameter('markers_topic', '/sc2/visual_servo/debug_markers')
        self.declare_parameter('use_grasp_pose', True)

        self.declare_parameter('image_width', 1280.0)
        self.declare_parameter('image_height', 720.0)
        self.declare_parameter('target_u', 640.0)
        self.declare_parameter('target_v', 360.0)

        self.declare_parameter('body_frame_id', 'base_link')
        self.declare_parameter('panel_x_m', 1.0)
        self.declare_parameter('panel_scale_m', 1.4)
        self.declare_parameter('cmd_scale_m', 1.8)
        self.declare_parameter('update_hz', 15.0)
        self.declare_parameter('timeout_ms', 1200.0)
        self.declare_parameter('marker_lifetime_sec', 0.6)

        self._target_topic = str(self.get_parameter('target_topic').value)
        self._cmd_topic = str(self.get_parameter('cmd_topic').value)
        self._grasp_pose_topic = str(self.get_parameter('grasp_pose_topic').value)
        self._markers_topic = str(self.get_parameter('markers_topic').value)
        self._use_grasp_pose = self._as_bool(self.get_parameter('use_grasp_pose').value)

        self._img_w = float(self.get_parameter('image_width').value)
        self._img_h = float(self.get_parameter('image_height').value)
        self._u0 = float(self.get_parameter('target_u').value)
        self._v0 = float(self.get_parameter('target_v').value)

        self._body_frame_id = str(self.get_parameter('body_frame_id').value)
        self._panel_x = float(self.get_parameter('panel_x_m').value)
        self._panel_scale = float(self.get_parameter('panel_scale_m').value)
        self._cmd_scale = float(self.get_parameter('cmd_scale_m').value)
        self._timeout_sec = float(self.get_parameter('timeout_ms').value) * 1e-3
        self._marker_lifetime_sec = max(0.0, float(self.get_parameter('marker_lifetime_sec').value))

        self._latest_target: Optional[TargetObservation] = None
        self._latest_cmd: Optional[TwistStamped] = None
        self._latest_grasp: Optional[PoseStamped] = None
        self._last_target_time = self.get_clock().now()
        self._last_cmd_time = self.get_clock().now()
        self._last_grasp_time = self.get_clock().now()

        self._pub = self.create_publisher(MarkerArray, self._markers_topic, 10)
        self.create_subscription(TargetObservation, self._target_topic, self._target_cb, 20)
        self.create_subscription(TwistStamped, self._cmd_topic, self._cmd_cb, 20)
        if self._use_grasp_pose:
            self.create_subscription(PoseStamped, self._grasp_pose_topic, self._grasp_cb, 20)

        hz = max(1e-3, float(self.get_parameter('update_hz').value))
        self.create_timer(1.0 / hz, self._tick)

        self.get_logger().info(
            f'visual_servo_debug_viz_node: target={self._target_topic}, cmd={self._cmd_topic}, '
            f'markers={self._markers_topic}'
        )

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {'1', 'true', 'yes', 'on'}
        return bool(value)

    def _target_cb(self, msg: TargetObservation) -> None:
        self._latest_target = msg
        self._last_target_time = self.get_clock().now()

    def _cmd_cb(self, msg: TwistStamped) -> None:
        self._latest_cmd = msg
        self._last_cmd_time = self.get_clock().now()

    def _grasp_cb(self, msg: PoseStamped) -> None:
        self._latest_grasp = msg
        self._last_grasp_time = self.get_clock().now()

    def _is_fresh(self, last_time) -> bool:
        return (self.get_clock().now() - last_time).nanoseconds * 1e-9 <= self._timeout_sec

    @staticmethod
    def _make_point(x: float, y: float, z: float) -> Point:
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    @staticmethod
    def _set_rgba(marker: Marker, r: float, g: float, b: float, a: float = 1.0) -> None:
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = float(a)

    def _mk_base_marker(self, frame_id: str, ns: str, marker_id: int, marker_type: int) -> Marker:
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = frame_id
        m.ns = ns
        m.id = int(marker_id)
        m.type = marker_type
        m.action = Marker.ADD
        if self._marker_lifetime_sec > 0.0:
            sec = int(self._marker_lifetime_sec)
            nanosec = int((self._marker_lifetime_sec - sec) * 1e9)
            m.lifetime.sec = sec
            m.lifetime.nanosec = nanosec
        return m

    def _tick(self) -> None:
        markers = MarkerArray()

        # Command vector marker in body frame.
        if self._latest_cmd is not None and self._is_fresh(self._last_cmd_time):
            vx = float(self._latest_cmd.twist.linear.x)
            vy = float(self._latest_cmd.twist.linear.y)
            vz = float(self._latest_cmd.twist.linear.z)
            yaw = float(self._latest_cmd.twist.angular.z)

            arrow = self._mk_base_marker(self._body_frame_id, 'vs_cmd', 0, Marker.ARROW)
            arrow.scale.x = 0.03
            arrow.scale.y = 0.06
            arrow.scale.z = 0.08
            self._set_rgba(arrow, 1.0, 0.2, 0.2, 0.95)
            arrow.points = [
                self._make_point(0.0, 0.0, 0.0),
                self._make_point(vx * self._cmd_scale, vy * self._cmd_scale, vz * self._cmd_scale),
            ]
            markers.markers.append(arrow)

            text = self._mk_base_marker(self._body_frame_id, 'vs_cmd', 1, Marker.TEXT_VIEW_FACING)
            text.pose.position = self._make_point(0.0, 0.0, 0.35)
            text.scale.z = 0.12
            self._set_rgba(text, 1.0, 1.0, 1.0, 0.95)
            text.text = f'cmd vx={vx:+.2f} vy={vy:+.2f} vz={vz:+.2f} yaw={yaw:+.2f}'
            markers.markers.append(text)

        # BBox marker projected onto a small debug panel in body frame.
        if self._latest_target is not None and self._is_fresh(self._last_target_time):
            t = self._latest_target
            ex = (float(t.u) - self._u0) / max(1.0, self._img_w)
            ey = (float(t.v) - self._v0) / max(1.0, self._img_h)

            py = ex * self._panel_scale
            pz = -ey * self._panel_scale
            half_w = 0.5 * (float(t.w) / max(1.0, self._img_w)) * self._panel_scale
            half_h = 0.5 * (float(t.h) / max(1.0, self._img_h)) * self._panel_scale

            center = self._mk_base_marker(self._body_frame_id, 'vs_bbox', 10, Marker.SPHERE)
            center.pose.position = self._make_point(self._panel_x, py, pz)
            center.scale.x = 0.08
            center.scale.y = 0.08
            center.scale.z = 0.08
            self._set_rgba(center, 0.1, 0.95, 0.1, 0.95)
            markers.markers.append(center)

            rect = self._mk_base_marker(self._body_frame_id, 'vs_bbox', 11, Marker.LINE_STRIP)
            rect.scale.x = 0.02
            self._set_rgba(rect, 0.1, 0.9, 1.0, 0.95)
            rect.points = [
                self._make_point(self._panel_x, py - half_w, pz - half_h),
                self._make_point(self._panel_x, py + half_w, pz - half_h),
                self._make_point(self._panel_x, py + half_w, pz + half_h),
                self._make_point(self._panel_x, py - half_w, pz + half_h),
                self._make_point(self._panel_x, py - half_w, pz - half_h),
            ]
            markers.markers.append(rect)

            cross = self._mk_base_marker(self._body_frame_id, 'vs_bbox', 12, Marker.LINE_LIST)
            cross.scale.x = 0.02
            self._set_rgba(cross, 1.0, 1.0, 0.1, 0.9)
            s = 0.08
            cross.points = [
                self._make_point(self._panel_x, -s, 0.0),
                self._make_point(self._panel_x, s, 0.0),
                self._make_point(self._panel_x, 0.0, -s),
                self._make_point(self._panel_x, 0.0, s),
            ]
            markers.markers.append(cross)

            target_text = self._mk_base_marker(self._body_frame_id, 'vs_bbox', 13, Marker.TEXT_VIEW_FACING)
            target_text.pose.position = self._make_point(self._panel_x + 0.08, py, pz + 0.1)
            target_text.scale.z = 0.10
            self._set_rgba(target_text, 0.95, 0.95, 0.95, 0.95)
            target_text.text = (
                f'bbox u={float(t.u):.0f} v={float(t.v):.0f} w={float(t.w):.0f} h={float(t.h):.0f} '
                f'd={float(t.depth_m):.2f}'
            )
            markers.markers.append(target_text)

        if self._use_grasp_pose and self._latest_grasp is not None and self._is_fresh(self._last_grasp_time):
            gp = self._latest_grasp
            grasp_sphere = self._mk_base_marker(gp.header.frame_id, 'vs_grasp', 20, Marker.SPHERE)
            grasp_sphere.pose.position = gp.pose.position
            grasp_sphere.scale.x = 0.06
            grasp_sphere.scale.y = 0.06
            grasp_sphere.scale.z = 0.06
            self._set_rgba(grasp_sphere, 1.0, 0.5, 0.1, 0.95)
            markers.markers.append(grasp_sphere)

        if markers.markers:
            self._pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoDebugVizNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
