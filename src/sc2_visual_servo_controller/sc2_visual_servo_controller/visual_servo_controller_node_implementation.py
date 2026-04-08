from typing import Optional
import numpy as np

from geometry_msgs.msg import TwistStamped, PoseStamped
from vision_msgs.msg import Detection2D
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from enum import Enum

import tf2_ros
import tf2_geometry_msgs 
from geometry_msgs.msg import Vector3Stamped

from .ibvs.controller_math import (
    clamp,
    is_in_roi,
    get_image_jacobian,
    get_uncalibrated_jacobian
)

class VisualServoControllerNode(Node):
    
    # state machine
    class ControllerState(Enum):
        BREAK = 1
        MANUAL = 2          # TODO:Implement Manual Mode as Control via the Joystick
        VISUAL = 3
        DVL = 4 
        COMPLETE = 5
    
    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {'1', 'true', 'yes', 'on'}
        return bool(value)

    def __init__(self):
        super().__init__('visual_servo_controller_node')

        self.declare_parameter('target_topic', '/sc2/visual_servo/target_obs')
        self.declare_parameter('target_point_topic', '/sc2/visual_servo/target_point')
        self.declare_parameter('target_bounding_box', '/best_detection')
        self.declare_parameter('cmd_topic', '/mavros/setpoint_velocity/cmd_vel') #mavros commanded velocity topic taken from the mavros docs
        self.declare_parameter('velocity_optical_topic', '/sc2/visual_servo/velocity_optical')
        self.declare_parameter('vel_measurement_topic', '/mavros/local_position/velocity_body')
        self.declare_parameter('camera_info_topic', '/oak/rgb/camera_info')
        self.declare_parameter('status_topic', '/sc2/visual_servo/status')
        self.declare_parameter('target_locked_state', '/sc2/visual_servo/target_locked')
        self.declare_parameter('body_frame_id', 'base_link')    # body frame of the BlueROV2
        self.declare_parameter('camera_optical_frame_id', 'oak_rgb_camera_optical_frame')    # body frame of the BlueROV2
        self.declare_parameter('timeout_ms', 500)
        self.declare_parameter('target_point_timeout_ms', 500)
        self.declare_parameter('rate_hz', 60.0)
        self.declare_parameter('controller_mode', 'ibvs')
        self.declare_parameter('calibrated', True)
        self.declare_parameter('verbose', True)
        self.declare_parameter('enable_dvl', False)
        self.declare_parameter('debug_state_machine', True)

        self.declare_parameter('image_width', 1280.0)
        self.declare_parameter('image_height', 720.0)
        self.declare_parameter('target_u', 640.0)
        self.declare_parameter('target_v', 360.0)
        self.declare_parameter('desired_depth_m', 1.0)

        self.declare_parameter('desired_target_distance_m', 0.6)
        self.declare_parameter('vertical_gain', 1.0)

        self.declare_parameter('max_vx', 0.4)
        self.declare_parameter('max_vy', 0.3)
        self.declare_parameter('max_vz', 0.3)
        self.declare_parameter('max_yaw_rate', 0.4)
        self.declare_parameter('slew_vx', 2.0)
        self.declare_parameter('slew_vy', 2.0)
        self.declare_parameter('slew_vz', 2.0)
        self.declare_parameter('slew_yaw_rate', 2.0)

        self.declare_parameter('fx_px', 600.0)
        self.declare_parameter('fy_px', 600.0)
        self.declare_parameter('cx_px', 640.0)
        self.declare_parameter('cy_px', 360.0)

        self._target_bbox_topic = str(self.get_parameter('target_bounding_box').value)
        self._target_pose_topic = str(self.get_parameter('target_point_topic').value)
        self._target_locked_topic = str(self.get_parameter('target_locked_state').value)
        self._camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self._cmd_topic = str(self.get_parameter('cmd_topic').value)
        self._velocity_optical_topic = str(self.get_parameter('velocity_optical_topic').value)
        self._velocity_measurement_topic = str(self.get_parameter('vel_measurement_topic').value)
        self._status_topic = str(self.get_parameter('status_topic').value)
        self._body_frame_id = str(self.get_parameter('body_frame_id').value)
        self._camera_optical_frame_id = str(self.get_parameter('camera_optical_frame_id').value)
        self._timeout_sec = float(self.get_parameter('timeout_ms').value) * 1e-3
        self._target_point_timeout_sec = float(self.get_parameter('target_point_timeout_ms').value) * 1e-3
        self._controller_mode = str(self.get_parameter('controller_mode').value).strip().lower()
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        self.enable_dvl = self.get_parameter('enable_dvl').get_parameter_value().bool_value
        self.debug_state_machine = self.get_parameter('debug_state_machine').get_parameter_value().bool_value

        self._max_vx = float(self.get_parameter('max_vx').value)
        self._max_vy = float(self.get_parameter('max_vy').value)
        self._max_vz = float(self.get_parameter('max_vz').value)
        self._max_yaw = float(self.get_parameter('max_yaw_rate').value)
        self._slew_vx = float(self.get_parameter('slew_vx').value)
        self._slew_vy = float(self.get_parameter('slew_vy').value)
        self._slew_vz = float(self.get_parameter('slew_vz').value)
        self._slew_yaw = float(self.get_parameter('slew_yaw_rate').value)

        self.calibrated = bool(self.get_parameter('calibrated').value)
        self._fx = float(self.get_parameter('fx_px').value)
        self._fy = float(self.get_parameter('fy_px').value)
        self._cx = float(self.get_parameter('cx_px').value)
        self._cy = float(self.get_parameter('cy_px').value)
        self.image_height = float(self.get_parameter('image_height').value)
        self.image_width = float(self.get_parameter('image_width').value)
        self._desired_target_dist = float(self.get_parameter('desired_target_distance_m').value)

        self._latest_obs: Optional[Detection2D] = None

        self.first_execution = True
        self._last_obs_time = self.get_clock().now()
        self._last_target_point_time = self.get_clock().now()
        self._last_tick_time = self.get_clock().now()
        self._last_cmd = (0.0, 0.0, 0.0, 0.0)
        self.target_count = 0
        self.center_count = 0
        self._last_debug_snapshot = None

        ###########################################################################################################################################
        ################################## Here start the IBVS control parameters which were validated in simulation ##############################
        ###########################################################################################################################################

        # target paratmers
        # first define the corners of a sqaure around the center of the image
        # TODO: validate whether these target corners suffice 
        side_length = 100
        h = side_length / 2.0
        cx, cy = self.image_height / 2.0, self.image_height / 2.0

        self.center_box_corners = np.array([
            [cx - h, cy - h],  # Top-Left
            [cx + h, cy - h],  # Top-Right
            [cx + h, cy + h],  # Bottom-Right
            [cx - h, cy + h]   # Bottom-Left
        ], dtype=np.float32)

        self.target_centroid = np.mean(self.center_box_corners, axis=1)
        self.prev_centroid = np.zeros(2)
        self.roi_w = 50

        # define the minimum needed detections and the detection counter
        self.confirmation_time = 0.1  # Must see target for 0.1s to go VISUAL
        self.lost_timeout = 0.2        # Must lose target for 0.2s to go BREAK
        self._first_seen_time = None
        self.target_min = 10
        self.center_min = 10

        # IBVS parameters
        self.lmbda = 0          # the lambda control gain from the standard IBVS control law
        self.lmbda_max = 0.1    # the maximum control gain that we can apply
        self.v_prev = np.zeros(6)
        self.alpha_1 = 0.6
        self.alpha_2 = 0.1
        self.v_last_valid = np.zeros(6)
        self.decay_rate = 0.95  # Drops 5% of speed per "blind" frame
        self.DoF_mask = [1, 1, 1, 1, 1, 1]
        self.latest_velocity_measurement = np.zeros(6)  # a variable to store the incoming velocity measurements from mavros/pixhawk
        self.prev_velocity_measurement = np.zeros(6)    # keep the previous velocity measurement for estimating travelled distance in the uncalibrated case
        self.current_e = np.zeros(6)
        self.J_img = np.eye(2*self.center_box_corners.shape[0], 6)  # initialise the Jacobian based on the number of target points

        # DVL control parameters
        self._latest_depth_measurement = 3.0
        self.dist_target = float(self.get_parameter('desired_target_distance_m').value) # desired vertical distance to target     
        self.kz = float(self.get_parameter('vertical_gain').value) # vertical gain

        # uncalibrated visual servoing
        self.Pk = np.eye(6) * 0.1
        self.lmbda_rls = 0.6
        self.prev_e = np.zeros_like(self.center_box_corners).flatten(order="F") 
        self.delta_e = 0

        #######################################################################################################################################################
        ###################### Here end the IBVS controller parameters which were validated in simulation #####################################################
        #######################################################################################################################################################

        self._cmd_pub = self.create_publisher(TwistStamped, self._cmd_topic, 10)
        self._optical_vel_pub = self.create_publisher(TwistStamped, self._velocity_optical_topic, 10)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._target_locked_pub = self.create_publisher(String, self._target_locked_topic, 10)
        self.create_subscription(Detection2D, self._target_bbox_topic, self._detection_cb, 10)
        self.create_subscription(CameraInfo, self._camera_info_topic, self._camera_info_cb, 10)
        self.create_subscription(TwistStamped, self._velocity_measurement_topic, self._vel_measurement_cb, 10)
        self.create_subscription(PoseStamped, self._target_pose_topic, self._target_pose_cb, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        rate_hz = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / max(1e-3, rate_hz), self._tick)

        self.current_state = self.ControllerState.BREAK
        self._publish_status(str(self.current_state.name))
        self._log_subscription_summary()

        self.get_logger().info(
            f'visual_servo_controller_node mode={self._controller_mode} target={self._target_bbox_topic} cmd={self._cmd_topic}'
        )

    def _target_pose_cb(self, msg: PoseStamped) -> None:
        '''Extract the depth information from the latest Pose etimate.'''
        latest_pose = msg.pose
        self._latest_depth_measurement = latest_pose.position.z

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        '''
        Extract CameraInfo paramters from the CameraInfo topic.
            [fx  0 cx]
        K = [ 0 fy cy]
            [ 0  0  1]
        '''
        _camera_info_msg = msg
        k = _camera_info_msg.k  # extract the intrinsic camera matrix
        self.fx = k[0]
        self.fy = k[4]
        self.cx = k[2]
        self.cy = k[5]

    def _log_subscription_summary(self) -> None:
        self.get_logger().info(
            'Subscriptions: '
            f'bbox={self._target_bbox_topic}, '
            f'cam_info={self._camera_info_topic}, '
            f'vel={self._velocity_measurement_topic}, '
            f'pose_topic_param={self._target_pose_topic}, '
            f'pose_cb_sub={self._target_pose_topic}'
        )

    def _log_debug_snapshot(
        self,
        *,
        dt_obs: float,
        is_fresh: bool,
        in_roi: bool,
        is_locked: bool,
    ) -> None:
        if not self.debug_state_machine:
            return

        snapshot = (
            self.current_state.name,
            round(dt_obs, 3),
            is_fresh,
            in_roi,
            is_locked,
            self.target_count,
            self.center_count,
        )

        changed = snapshot != self._last_debug_snapshot
        if changed:
            self._last_debug_snapshot = snapshot

        message = (
            'Debug: '
            f'state={self.current_state.name} '
            f'dt_obs={dt_obs:.2f} '
            f'is_fresh={is_fresh} '
            f'in_roi={in_roi} '
            f'locked={is_locked} '
            f'target_count={self.target_count} '
            f'center_count={self.center_count}'
        )
        if changed:
            self.get_logger().info(message)
        else:
            self.get_logger().info(message, throttle_duration_sec=1.0)
    
    def _vel_measurement_cb(self, msg: TwistStamped) -> None:
        t = msg.twist
        current_vel = np.array([
            t.linear.x,  t.linear.y,  t.linear.z,
            t.angular.x, t.angular.y, t.angular.z
        ])
        self.latest_velocity_measurement = current_vel

    def _detection_cb(self, detection_2D_msg: Detection2D):
        """This callback reads an incoming detection from the object detection model, extracts the detection with the desired class_id
        with the highest confidence, and then extracts it"s bounding box.
        
        Args:
        detection_array_msg (Detection2DArray) : Array with detection messages

        Returns:
        determines which control method to use, and computes corresponding errors for the PID control signals    

        """
        
        # self.get_logger().info(
        #     f'\nReceived a position of x={position.x:6.2f}, y={position.y:6.2f}\n'
        #     f'position setpoint at x={self.setpoint_x:6.2f}, y={self.setpoint_y:6.2f}',
        #     throttle_duration_sec=1,
        # )

        self.get_logger().info(f'Detection Recieved!', throttle_duration_sec=2.0)

        self._last_obs_time = self.get_clock().now()
        self._latest_obs = detection_2D_msg.bbox
        best_bbox = self._latest_obs
        
        half_w = best_bbox.size_x / 2.0
        half_h = best_bbox.size_y / 2.0

        # Define corners (Top-Left, Top-Right, Bottom-Right, Bottom-Left)
        self.latest_bbox_corners = [
            (best_bbox.center.position.x - half_w, best_bbox.center.position.y - half_h), # TL
            (best_bbox.center.position.x + half_w, best_bbox.center.position.y - half_h), # TR
            (best_bbox.center.position.x + half_w, best_bbox.center.position.y + half_h), # BR
            (best_bbox.center.position.x - half_w, best_bbox.center.position.y + half_h)  # BL
        ]

        self.latest_centroid = np.mean(self.latest_bbox_corners, axis=1).copy()

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt_obs = (now - self._last_obs_time).nanoseconds * 1e-9
        dt_ctrl = max(1e-4, (now - self._last_tick_time).nanoseconds * 1e-9)
        self._last_tick_time = now

        # --- 1. SENSOR VALIDATION GATE ---
        is_fresh = self._latest_obs is not None and dt_obs <= self._timeout_sec
        
        # ROI check against the LAST VALID centroid we trusted
        in_roi = True
        if not self.first_execution:
            in_roi = is_in_roi(self.latest_centroid[0], self.latest_centroid[1], self.prev_centroid, self.roi_w)
            self.prev_centroid = self.latest_centroid.copy()

        # Update the Consecutive Counter
        if is_fresh and in_roi:
            # We only count it as a "valid frame" if it's fresh and near the last one
            self.target_count = min(self.target_count + 1, self.target_min + 1)
        else:
            # Hard reset: If it jumps or times out, we lose the "lock"
            self.target_count = 0

        # Boolean for whether we trust the current frame enough to do math
        is_locked = (self.target_count > self.target_min)

        # publish if the target is locked or not for debugging
        self._publish_is_locked(str(is_locked))
        self._log_debug_snapshot(
            dt_obs=dt_obs,
            is_fresh=is_fresh,
            in_roi=in_roi,
            is_locked=is_locked,
        )

        # --- 2. MATH UPDATE (Only if Locked) ---
        if is_locked:
            self.current_e = (self.latest_bbox_corners - self.center_box_corners).flatten(order="F") 
            Z = np.ones(len(self.latest_bbox_corners)) * self._latest_depth_measurement # apply the latest depth measurement of the bbbox center

            if self.calibrated:
                self.J_img = get_image_jacobian(self.latest_bbox_corners, Z, self.fx, self.fy, self.cx, self.cy)
            else:
                # UNCALIBRATED MODE: Update RLS Jacobian estimate
                self.J_hat, self.Pk, self.delta_e = get_uncalibrated_jacobian(
                    self.current_e, self.prev_e, self.prev_velocity_measurement, 
                    self.J_hat, self.Pk, self.lmbda_rls, dt_ctrl
                )
                self.J_img = self.J_hat

            # Update history for next iteration
            self.prev_e = self.current_e.copy()
            self.prev_velocity_measurement = self.latest_velocity_measurement
            self.first_execution = False

        # --- 3. STATE TRANSITIONS ---
        if is_locked:
            # Check for VISUAL transition
            if self.current_state not in [self.ControllerState.DVL, self.ControllerState.COMPLETE]:
                self.set_state(self.ControllerState.VISUAL)

            # Check for DVL transition (Requires being in the center)
            in_center = is_in_roi(self.latest_centroid[0], self.latest_centroid[1], self.target_centroid, self.roi_w/2)
            if in_center and self.enable_dvl:
                self.center_count = min(self.center_count + 1, self.center_min + 1)
                if self.center_count > self.center_min:
                    self.set_state(self.ControllerState.DVL)
            else:
                self.center_count = 0
        else:
            # Not locked: if blind for too long, break
            if dt_obs >= self.lost_timeout:
                self.set_state(self.ControllerState.BREAK)

        # --- 4. ACTION SELECTION (Mode-Specific) ---
        # Initialize raw target velocity
        v_raw_target = np.zeros(6)

        if self.current_state == self.ControllerState.DVL:
            # DVL DESCENT LOGIC
            # error_z calculation (you'll need to ensure you have the Z-measurement available)
            error_z = 1.0 - self.dist_target # Placeholder: Replace with real altitude
            v_z = self.kz * error_z
            v_raw_target = np.array([0, 0, v_z, 0, 0, 0])
            
            if abs(error_z) < 0.05:
                self.set_state(self.ControllerState.COMPLETE)

        elif self.current_state == self.ControllerState.VISUAL and is_locked:
            # IBVS TRACKING LOGIC
            try:
                # Use Damped Least Squares inverse
                inv_J = np.linalg.inv(self.J_img.T @ self.J_img + 1e-4 * np.eye(6)) @ self.J_img.T
                v_raw_target = (-self.lmbda_max * inv_J @ self.current_e) * self.DoF_mask
            except np.linalg.LinAlgError:
                v_raw_target = self.v_last_valid * self.decay_rate
        
        else:
            # BREAK or GLIDE (Target lost while in VISUAL)
            v_raw_target = self.v_last_valid * self.decay_rate

        # --- 5. SMOOTHING & PUBLISHING ---
        # Apply EMA filter (only if target is valid, otherwise use raw decay)
        if is_locked or self.current_state == self.ControllerState.DVL:
            v_desired = (self.alpha_1 * v_raw_target) + ((1 - self.alpha_1) * self.v_last_valid)
        else:
            v_desired = v_raw_target # Already contains the decayed v_last_valid

        self.v_last_valid = v_desired
        vx, vy, vz, _, _, yaw = v_desired.flatten()

        # 1. Map IBVS output to the Camera's Optical Frame (Z-forward, Y-down)
        # Based on your cable (Y) and down (Z) observations:
        v_opt_msg = TwistStamped()
        v_opt_msg.header.stamp = self.get_clock().now().to_msg()
        v_opt_msg.header.frame_id = self._camera_optical_frame_id
        
        v_opt_msg.twist.linear.x = -vx    # Right
        v_opt_msg.twist.linear.y = vz    # Down
        v_opt_msg.twist.linear.z = -vy   # Forward (Away from cable)
        v_opt_msg.twist.angular.y = yaw  # Optical Yaw is around Y-axis

        # PUBLISH the optical debug velocity
        self._optical_vel_pub.publish(v_opt_msg)

        # 2. Prepare for TF Transformation (Linear and Angular separately)
        v_lin_opt = Vector3Stamped()
        v_lin_opt.header = v_opt_msg.header
        v_lin_opt.vector = v_opt_msg.twist.linear

        v_ang_opt = Vector3Stamped()
        v_ang_opt.header = v_opt_msg.header
        v_ang_opt.vector = v_opt_msg.twist.angular

        try:
            # Look up the transform from Optical Frame to ROV Body Frame
            transform = self.tf_buffer.lookup_transform(
                self._body_frame_id, 
                self._camera_optical_frame_id, 
                rclpy.time.Time()
            )
            
            # Transform both vectors to the ROV's coordinate system
            v_lin_body = tf2_geometry_msgs.do_transform_vector3(v_lin_opt, transform)
            v_ang_body = tf2_geometry_msgs.do_transform_vector3(v_ang_opt, transform)
            
            # 3. Publish to BlueROV2 (MAVROS)
            # We take the resulting Z-axis rotation as the ROV's Yaw
            self._publish_cmd_with_slew(
                v_lin_body.vector.x, 
                v_lin_body.vector.y, 
                v_lin_body.vector.z, 
                v_ang_body.vector.z, 
                dt_ctrl
            )

        except Exception as e:
            self.get_logger().error(f"TF Transform failed: {e}", throttle_duration_sec=1.0)

    def set_state(self, new_state):
        if new_state != self.current_state:
            # Logging the change
            self.get_logger().info(f"State transition: {self.current_state.name} -> {new_state.name}")
            
            # Specific transition logic
            if new_state == self.ControllerState.DVL:
                if self.verbose:
                    self.get_logger().info("DVL Latch: Resetting horizontal velocity filters.")
                self.v_last_valid[0] = 0
                self.v_last_valid[1] = 0
                self.v_last_valid[5] = 0
                
                self.v_prev[0] = 0
                self.v_prev[1] = 0
                self.v_prev[5] = 0

            # Update the variable
            self.current_state = new_state
            self._last_debug_snapshot = None
            
            # PUBLISH the new state name
            self._publish_status(str(self.current_state.name))
    
    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _publish_is_locked(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._target_locked_pub.publish(msg)

    def _publish_cmd(self, vx: float, vy: float, vz: float, yaw: float) -> None:
        vx = clamp(vx, self._max_vx)
        vy = clamp(vy, self._max_vy)
        vz = clamp(vz, self._max_vz)
        yaw = clamp(yaw, self._max_yaw)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self._body_frame_id
        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        cmd.twist.linear.z = vz
        cmd.twist.angular.z = yaw
        self._cmd_pub.publish(cmd)
        self._last_cmd = (vx, vy, vz, yaw)

    @staticmethod
    def _slew_axis(target: float, prev: float, limit_per_s: float, dt: float) -> float:
        if limit_per_s <= 0.0 or dt <= 0.0:
            return target
        delta_max = limit_per_s * dt
        delta = target - prev
        if delta > delta_max:
            return prev + delta_max
        if delta < -delta_max:
            return prev - delta_max
        return target

    def _publish_cmd_with_slew(self, vx: float, vy: float, vz: float, yaw: float, dt: float) -> None:
        pvx, pvy, pvz, pyaw = self._last_cmd
        svx = self._slew_axis(vx, pvx, self._slew_vx, dt)
        svy = self._slew_axis(vy, pvy, self._slew_vy, dt)
        svz = self._slew_axis(vz, pvz, self._slew_vz, dt)
        syaw = self._slew_axis(yaw, pyaw, self._slew_yaw, dt)
        self._publish_cmd(svx, svy, svz, syaw)

def main(args=None):
    rclpy.init(args=args)
    node = VisualServoControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
