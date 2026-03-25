from typing import Optional
import numpy as np

from geometry_msgs.msg import TwistStamped
from vision_msgs.msg import Detection2D
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from enum import Enum

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
        self.declare_parameter('vel_measurement_topic', '/mavros/local_position/velocity_body')
        self.declare_parameter('camera_info_topic', '/oak/rgb/camera_info')
        self.declare_parameter('status_topic', '/sc2/visual_servo/status')
        self.declare_parameter('body_frame_id', 'base_link')    # body frame of the BlueROV2
        self.declare_parameter('timeout_ms', 500)
        self.declare_parameter('target_point_timeout_ms', 500)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('controller_mode', 'ibvs')
        self.declare_parameter('calibrated', True)
        self.declare_parameter('verbose', True)
        self.declare_parameter('enable_dvl', False)

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

        self._target_topic = str(self.get_parameter('target_bounding_box').value)
        self._camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self._cmd_topic = str(self.get_parameter('cmd_topic').value)
        self._velocity_measurement_topic = str(self.get_parameter('vel_measurement_topic').value)
        self._status_topic = str(self.get_parameter('status_topic').value)
        self._body_frame_id = str(self.get_parameter('body_frame_id').value)
        self._timeout_sec = float(self.get_parameter('timeout_ms').value) * 1e-3
        self._target_point_timeout_sec = float(self.get_parameter('target_point_timeout_ms').value) * 1e-3
        self._controller_mode = str(self.get_parameter('controller_mode').value).strip().lower()
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        self.enable_dvl = self.get_parameter('enable_dvl').get_parameter_value().bool_value

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

        ###########################################################################################################################################
        ################################## Here start the IBVS control parameters which were validated in simulation ##############################
        ###########################################################################################################################################

        # IBVS parameters
        self.lmbda = 0          # the lambda control gain from the standard IBVS control law
        self.lmbda_max = 0.1    # the maximum control gain that we can apply
        self.v_prev = np.zeros(6)
        self.alpha_1 = 0.6
        self.alpha_2 = 0.1
        self.v_last_valid = np.zeros(6)
        self.decay_rate = 0.15  # Drops 5% of speed per "blind" frame
        self.DoF_mask = [1, 1, 1, 1, 1, 1]
        self.latest_velocity_measurement = np.zeros(6)  # a variable to store the incoming velocity measurements from mavros/pixhawk
        self.prev_velocity_measurement = np.zeros(6)    # keep the previous velocity measurement for estimating travelled distance in the uncalibrated case
        
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
        self.target_min = 5
        self.center_min = 5
        self.target_count = 0
        self.center_count = 0

        # DVL control parameters
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
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self.create_subscription(Detection2D, self._target_topic, self._detection_cb, 10)
        self.create_subscription(CameraInfo, self._camera_info_topic, self._camera_info_cb, 10)
        self.create_subscription(TwistStamped, self._velocity_measurement_topic, self._vel_measurement_cb, 10)

        rate_hz = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / max(1e-3, rate_hz), self._tick)

        self.current_state = self.ControllerState.BREAK
        self._publish_status(self.current_state.name)

        self.get_logger().info(
            f'visual_servo_controller_node mode={self._controller_mode} target={self._target_topic} cmd={self._cmd_topic}'
        )

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
        dt_target_point = (now - self._last_target_point_time).nanoseconds * 1e-9
        dt_ctrl = max(1e-4, (now - self._last_tick_time).nanoseconds * 1e-9)
        self._last_tick_time = now

        if self._latest_obs is not None and not dt_obs > self._timeout_sec:
            # compute the error of the bounding box corners to the target box corners
            e = (self.latest_bbox_corners - self.center_box_corners).flatten(order="F") 

            # TODO: assume a constant depth for now
            Z = np.ones(len(self.latest_bbox_corners)) * 3.0

            # for the uncalibrated case, initialize the Jacobian estimate with the current camera parameters
            if self.first_execution and not self.calibrated:
                self.J_hat = get_image_jacobian(self.latest_bbox_corners, Z, self.fx.copy(), self.fy.copy(), self.cx.copy(), self.cy.copy())
                self.first_execution = False

            # compute the desired raw velocity based on the image Jacobian 
            if self.calibrated:
                J_img = get_image_jacobian(self.latest_bbox_corners, Z, self.fx.copy(), self.fy.copy(), self.cx.copy(), self.cy.copy())
            else:
                self.J_hat, self.Pk, self.delta_e = get_uncalibrated_jacobian(e, self.prev_e, self.prev_velocity_measurement, self.J_hat, self.Pk, self.lmbda_rls, self.dt_ctrl)
                J_img = self.J_hat

            self.prev_e = e.copy()
            self.prev_velocity_measurement = self.latest_velocity_measurement

        else:
            self.get_logger().info(f"No detections yet, cannot compute control signal...", throttle_duration_sec=2.0)
            return
        
        # compute the commanded velocity
        try:
            raw_v = -self.lmbda_max * np.linalg.inv(J_img.T @ J_img + 1e-4 * np.eye(6)) @ J_img.T @ e # use damped peudo-inverse
        except np.linalg.LinAlgError:
            return -1
        

        # check whether the current bbox is in the ROI of the previously detected bbox
        in_roi = is_in_roi(self.latest_centroid[0], self.latest_centroid[1], self.prev_centroid, self.roi_w/2)
        self.prev_centroid = self.latest_centroid

        # if there is a fresh observation, and 
        if self._latest_obs is not None or not dt_obs > self._timeout_sec and in_roi:
            if self.verbose:
                self.get_logger().info(f"Target Visible!", throttle_duration_sec=2.0)
            
            self.target_count += 1

            # Transition to VISUAL if we've seen it enough and aren't DVL yet
            if self.target_count > self.target_min and self.current_state not in [self.ControllerState.DVL, self.ControllerState.COMPLETE]:
                self.set_state(self.ControllerState.VISUAL)

            # Check if we should switch to DVL (Target is in the center ROI)
            in_center = is_in_roi(self.latest_centroid[0], self.latest_centroid[1], self.target_centroid, self.roi_w/2)
            if in_center and self.enable_dvl:  # Only transition if enabled
                if self.verbose:
                    self.get_logger().info(f"Target in Center ROI - Transitioning to DVL", throttle_duration_sec=2.0)
                self.center_count += 1
                if self.center_count > self.center_min:
                    self.set_state(self.ControllerState.DVL)
            elif in_center and not self.enable_dvl:
                if self.verbose:
                    self.get_logger().info(f"Target in Center but DVL disabled. Staying in VISUAL.", throttle_duration_sec=2.0)

        else:
            # We don't reset target_count immediately to 0; we let it "cool down"
            # This prevents 1-frame flickers from resetting the whole state
            self.get_logger().info(f"Target Not Visible!", throttle_duration_sec=2.0)
            self.target_count = max(0, self.target_count - 1) 
            self.center_count = max(0, self.center_count - 1)
            if self.target_count == 0 and self.current_state == self.ControllerState.VISUAL:
                self.set_state(self.ControllerState.BREAK)


        if self.current_state == self.ControllerState.DVL:
            if self.verbose:
                self.get_logger().info(f"DVL Control Mode: Descending...")
            # Calculate vertical distance to the target plane
            current_distance = np.abs(self.camera.pose.t[2] - self.P.T[0,2])
            error_z = current_distance - self.dist_target
            
            # Proportional descent + decay any existing horizontal drift
            v_z = self.kz * error_z
            v_drift = self.v_last_valid * self.decay_rate
            v_raw_target = np.array([0, 0, v_z, 0, 0, 0])
            
            if error_z < 0.05:
                self.set_state(self.ControllerState.COMPLETE)
                return 1

        elif self.current_state == self.ControllerState.VISUAL:
            # We are in tracking mode. If this specific frame is a flicker, decay.
            if self._latest_obs is not None or not dt_obs > self._timeout_sec and in_roi:
                v_raw_target = raw_v * self.DoF_mask # Surge, Sway, Yaw only
            else:
                self.set_state(self.ControllerState.BREAK)
                v_raw_target = self.v_last_valid * self.decay_rate
        
        else:
            # BREAK mode or "Warming up": apply decay to whatever the last velocity was
            self.set_state(self.ControllerState.BREAK)
            v_raw_target = self.v_last_valid * self.decay_rate

        # filter the velocity with a first-order low pass filter (Exponential Moving Average)
        v_desired = (self.alpha_1 * v_raw_target) + ((1 - self.alpha_1) * self.v_last_valid)
        self.v_last_valid = v_desired   # Save velocity for the next frame's decay calculation

        vx, vy, vz, _, _, yaw = v_desired.flatten()

        self.get_logger().info(f'Publishing Velocity: {v_desired.flatten()}', throttle_duration_sec=2.0)

        self._publish_cmd_with_slew(vx, vy, vz, yaw, dt_ctrl)
        self._publish_status('TRACKING_IBVS')

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
            
            # PUBLISH the new state name
            self._publish_status(self.current_state.name)
    
    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

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
