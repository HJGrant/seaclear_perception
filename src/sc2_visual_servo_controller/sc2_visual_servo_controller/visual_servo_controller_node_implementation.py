"""Visual servo controller node with legacy IBVS and Phase 2 staged PID paths.

Current staged PID behavior notes:
- Phase 2 controls body-frame x/y/yaw and can add z velocity after the
  horizontal gate is satisfied.
- `valid_target_observation` is only a tracking-continuity signal based on ROI
  consistency. It does not imply centered/aligned detection.
- `target_tracked` means the controller has a continuously trackable target-like
  observation. It does not imply horizontal lock or vertical-ready.
- Staged PID vertical defaults to target_point primary plus DVL fallback:
  `/sc2/visual_servo/target_point` (`PoseStamped`) is used while fresh; DVL
  altitude is used only when target_point is stale or unavailable. Fresh
  target_point always takes priority again when it recovers.
- `horizontal_aligned_now`, `vertical_enabled`, and `allow_z` gate z output.
  When `allow_z` is false, z velocity is forced to zero while horizontal control
  continues.
- Short stale gaps while `target_tracked` remains true decay the last horizontal
  command using `decay_rate`.
- The command adapter keeps the grapple IF and publishes both
  `TwistStamped` and `state_estimation/msg/Vector6`.
- `target_point.pose.position.z` is expected to be target-relative range in
  meters. Before hardware testing, still verify the resulting positive-z command
  convention against the vehicle frame.

Not implemented in this file yet: search scan/recovery and terminal hold/zero
behavior. The hold-distance check is only exposed as a status hook.
"""

from collections import deque
from typing import Optional
import numpy as np

from geometry_msgs.msg import TwistStamped, PoseStamped, PointStamped, Vector3, Vector3Stamped
from vision_msgs.msg import Detection2D
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from sensor_msgs.msg import CameraInfo
from enum import Enum
from std_srvs.srv import SetBool

import tf2_ros
import tf2_geometry_msgs 
from tf2_ros import TransformException
from state_estimation.msg import Vector6
from marine_acoustic_msgs.msg import Dvl

from .ibvs.controller_math import (
    clamp,
    is_in_roi,
    get_image_jacobian,
    get_uncalibrated_jacobian
)
from .pid import PIDController, PIDGains

class VisualServoControllerNode(Node):
    
    # state machine
    class ControllerState(Enum):
        BREAK = 1
        MANUAL = 2          # TODO:Implement Manual Mode as Control via the Joystick
        VISUAL = 3
        DVL = 4 
        COMPLETE = 5

    class StagedPIDState(Enum):
        HORIZONTAL = 'HORIZONTAL'
        HORIZONTAL_AND_VERTICAL = 'HORIZONTAL_AND_VERTICAL'
        NO_TARGET = 'NO_TARGET'
    
    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {'1', 'true', 'yes', 'on'}
        return bool(value)

    def __init__(self):
        super().__init__('visual_servo_controller_node')

        # relevant data topics
        self.declare_parameter('target_topic', '/sc2/visual_servo/target_obs')
        self.declare_parameter('target_point_topic', '/sc2/visual_servo/target_point')
        self.declare_parameter('target_bounding_box', '/best_detection')
        self.declare_parameter('cmd_topic', '/mavros/setpoint_velocity/cmd_vel') #mavros commanded velocity topic taken from the mavros docs
        self.declare_parameter('cmd_topic_vec_6', '/sc2/setpoint_velocity/cmd_vel_vec6') #the 
        self.declare_parameter('velocity_camera_image_plane_frame_topic', '/sc2/visual_servo/velocity_camera_image_plane_frame')    # DEBUG velocities published in the camera image plane frame 
        self.declare_parameter('velocity_sc2_base_link_topic', '/sc2/visual_servo/velocity_sc2_base_link')     # DEBUG velocities published in the reference sc2_base_link frame
        self.declare_parameter('vel_measurement_topic', '/mavros/local_position/velocity_body')
        self.declare_parameter('camera_info_topic', '/oak/rgb/camera_info')
        self.declare_parameter('status_topic', '/sc2/visual_servo/status')
        self.declare_parameter('target_locked_state', '/sc2/visual_servo/target_locked')
        self.declare_parameter('centroid_error_topic', '/sc2/visual_servo/centroid_error')
        self.declare_parameter('dvl_altitude_topic', '/sc2/velocity_report')
        self.declare_parameter('mavros_altitude_topic','/mavros/global_position/rel_alt')
        
        # relevant frames for transformations
        self.declare_parameter('mavros_base_link_frame_id', 'base_link')    # body frame used by mavros
        self.declare_parameter('sc2_base_link_frame_id', 'sc2_base_link')   # body frame used by us (SeaClear2)
        self.declare_parameter('body_frame_id', 'base_link')                # frame used for direct body-frame commands
        self.declare_parameter('camera_optical_frame_id', 'oak_rgb_camera_optical_frame')    # RGB camera optical frame
        self.declare_parameter('camera_image_plane_frame_id', 'image_plane')                # image plane frame

        # controller parameters 
        self.declare_parameter('timeout_ms', 500)
        self.declare_parameter('target_point_timeout_ms', 500)
        self.declare_parameter('rate_hz', 60.0)
        self.declare_parameter('controller_mode', 'ibvs')
        self.declare_parameter('calibrated', True)
        self.declare_parameter('verbose', True)
        self.declare_parameter('enable_dvl', False)
        self.declare_parameter('yaw_control', False)    #turn yaw control on/off
        self.declare_parameter('depth_estimation_mode', 'RGBD') # choose from: MAVROS, DVL or RGBD

        # image data parameters
        self.declare_parameter('image_width', 1280.0)
        self.declare_parameter('image_height', 640.0)
        self.declare_parameter('target_u', 640.0)
        self.declare_parameter('target_v', 360.0)

        # vertical control parameters
        self.declare_parameter('desired_depth_m', 1.0)
        self.declare_parameter('desired_target_distance_m', 0.6)
        self.declare_parameter('vertical_gain', 1.0)

        # controller parameters
        self.declare_parameter('max_vx', 0.4)
        self.declare_parameter('max_vy', 0.3)
        self.declare_parameter('max_vz', 0.3)
        self.declare_parameter('max_yaw_rate', 0.4)
        self.declare_parameter('slew_vx', 2.0)
        self.declare_parameter('slew_vy', 2.0)
        self.declare_parameter('slew_vz', 2.0)
        self.declare_parameter('slew_yaw_rate', 2.0)

        # camera parameters
        self.declare_parameter('fx_px', 600.0)
        self.declare_parameter('fy_px', 600.0)
        self.declare_parameter('cx_px', 640.0)
        self.declare_parameter('cy_px', 360.0)

        # staged PID parameters. Phase 2 adds target_pose-sourced vertical PID
        # while preserving Phase 1 horizontal behavior in HORIZONTAL.
        #
        # Vertical gate tolerance rationale:
        # With HFOV ~= 66 deg and the controller bbox image width set to 640 px,
        # at 0.5 m range 1 px is approximately 1 mm laterally. Therefore
        # 25 px ~= 2.5 cm, 150 px ~= 15 cm, and 200 px ~= 20 cm. The Phase 2
        # vertical gate uses 150 px on both u and v: slightly stricter than a
        # 0.2 m lateral-offset allowance at 0.5 m while avoiding the previous
        # 25 px near-perfect-centering assumption.
        self.declare_parameter('staged_pid_xy_kp', 0.0025)
        self.declare_parameter('staged_pid_xy_ki', 0.0)
        self.declare_parameter('staged_pid_xy_kd', 0.0008)
        self.declare_parameter('staged_pid_xy_integral_limit', 200.0)
        self.declare_parameter('staged_pid_z_kp', 0.8)
        self.declare_parameter('staged_pid_z_ki', 0.0)
        self.declare_parameter('staged_pid_z_kd', 0.10)
        self.declare_parameter('staged_pid_z_integral_limit', 1.0)
        self.declare_parameter('staged_pid_horizontal_tolerance_px', 150.0)
        self.declare_parameter('staged_pid_horizontal_tolerance_x_px', 150.0)
        self.declare_parameter('staged_pid_horizontal_tolerance_y_px', 150.0)
        self.declare_parameter('staged_pid_vertical_tolerance_px', 150.0)
        self.declare_parameter('staged_pid_distance_tolerance_m', 0.08)
        self.declare_parameter('staged_pid_vertical_gate_window_ticks', 10)
        self.declare_parameter('staged_pid_vertical_enter_ratio', 0.8)
        self.declare_parameter('staged_pid_vertical_exit_ratio', 0.4)
        self.declare_parameter('staged_pid_vertical_enter_hold_ticks', 2)
        self.declare_parameter('staged_pid_vertical_exit_hold_ticks', 2)
        self.declare_parameter('staged_pid_enable_yaw', False)
        self.declare_parameter('desired_vertical_distance_m', 0.6)
        self.declare_parameter('staged_pid_z_source', 'target_point_primary_dvl_fallback')
        self.declare_parameter('staged_pid_hold_zero_distance_m', 0.5)
        self.declare_parameter('tracked_enter_window_obs', 10)
        self.declare_parameter('tracked_enter_min_obs', 2)
        self.declare_parameter('tracked_exit_no_obs_sec', 0.6)
        
        # IBVS control parameters
        self.declare_parameter('ibvs_lambda', 0.2)
        self.declare_parameter('alpha_1', 0.6)
        self.declare_parameter('decay_rate', 0.95)

        # extract data topics
        self._target_bbox_topic = str(self.get_parameter('target_bounding_box').value)
        self._target_locked_topic = str(self.get_parameter('target_locked_state').value)
        self._camera_info_topic = str(self.get_parameter('camera_info_topic').value)

        self._cmd_topic = str(self.get_parameter('cmd_topic').value)
        self._sc2_cmd_topic_vec_6 = str(self.get_parameter('cmd_topic_vec_6').value)
        self._velocity_image_plane_frame_topic = str(self.get_parameter('velocity_camera_image_plane_frame_topic').value)
        self._velocity_sc2_base_link_topic = str(self.get_parameter('velocity_sc2_base_link_topic').value)
        
        self._velocity_measurement_topic = str(self.get_parameter('vel_measurement_topic').value)
        self._status_topic = str(self.get_parameter('status_topic').value)
        self._centroid_error_topic = str(self.get_parameter('centroid_error_topic').value)

        # extract frame IDs
        self._mavros_base_link_frame_id = str(self.get_parameter('mavros_base_link_frame_id').value)
        self._sc2_base_link_frame_id = str(self.get_parameter('sc2_base_link_frame_id').value)
        self._body_frame_id = str(self.get_parameter('body_frame_id').value)
        self._camera_optical_frame_id = str(self.get_parameter('camera_optical_frame_id').value)
        self._camera_image_plane_frame_id = str(self.get_parameter('camera_image_plane_frame_id').value)

        # extract parameters
        self._timeout_sec = float(self.get_parameter('timeout_ms').value) * 1e-3
        self._target_point_timeout_sec = float(self.get_parameter('target_point_timeout_ms').value) * 1e-3
        self._controller_mode = str(self.get_parameter('controller_mode').value).strip().lower()
        if self._controller_mode not in {'ibvs', 'staged_pid'}:
            self.get_logger().warn(f'Unknown controller_mode={self._controller_mode}; falling back to ibvs')
            self._controller_mode = 'ibvs'
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        self.enable_dvl = self.get_parameter('enable_dvl').get_parameter_value().bool_value
        self._yaw_control_switch = self.get_parameter('yaw_control').get_parameter_value().bool_value
        
        self._target_point_topic = str(self.get_parameter('target_point_topic').value)
        self._depth_mode = str(self.get_parameter('depth_estimation_mode').get_parameter_value().string_value)

        # which depth estimation mode to use for IBVS
        match self._depth_mode:
            case 'RGBD':
                self._target_pose_topic = self._target_point_topic
            case 'DVL':
                self._target_pose_topic = str(self.get_parameter('dvl_altitude_topic').value)
            case 'MAVROS':
                self._target_pose_topic = str(self.get_parameter('mavros_altitude_topic').value)

        self._max_vx = float(self.get_parameter('max_vx').value)
        self._max_vy = float(self.get_parameter('max_vy').value)
        self._max_vz = float(self.get_parameter('max_vz').value)
        self._max_yaw = float(self.get_parameter('max_yaw_rate').value)
        self._slew_vx = float(self.get_parameter('slew_vx').value)
        self._slew_vy = float(self.get_parameter('slew_vy').value)
        self._slew_vz = float(self.get_parameter('slew_vz').value)
        self._slew_yaw = float(self.get_parameter('slew_yaw_rate').value)

        # IBVS paramters
        self.ibvs_lmdba = float(self.get_parameter('ibvs_lambda').value)
        self.alpha_1_val = float(self.get_parameter('alpha_1').value)
        self.decay_rate_val = float(self.get_parameter('decay_rate').value)

        self.calibrated = bool(self.get_parameter('calibrated').value)
        self._fx = float(self.get_parameter('fx_px').value)
        self._fy = float(self.get_parameter('fy_px').value)
        self._cx = float(self.get_parameter('cx_px').value)
        self._cy = float(self.get_parameter('cy_px').value)
        self.image_height = float(self.get_parameter('image_height').value)
        self.image_width = float(self.get_parameter('image_width').value)
        self._target_u = float(self.get_parameter('target_u').value)
        self._target_v = float(self.get_parameter('target_v').value)
        self._desired_target_dist = float(self.get_parameter('desired_target_distance_m').value)
        self._desired_vertical_dist = float(self.get_parameter('desired_vertical_distance_m').value)
        self._staged_pid_z_source = str(self.get_parameter('staged_pid_z_source').value).strip().lower()
        if self._staged_pid_z_source in {'target_pose', 'target_point'}:
            self._staged_pid_z_source = 'target_point'
        elif self._staged_pid_z_source in {'target_point_primary_dvl_fallback', 'target_pose_primary_dvl_fallback'}:
            self._staged_pid_z_source = 'target_point_primary_dvl_fallback'
        elif self._staged_pid_z_source in {'dvl', 'dvl_fallback'}:
            self._staged_pid_z_source = 'dvl'
        else:
            self.get_logger().warn(
                f'Invalid staged_pid_z_source={self._staged_pid_z_source}; using target_point_primary_dvl_fallback'
            )
            self._staged_pid_z_source = 'target_point_primary_dvl_fallback'

        self._latest_obs: Optional[Detection2D] = None
        self._latest_target_point: Optional[PoseStamped] = None

        self.first_execution = True
        self._last_obs_time = self.get_clock().now()
        self._last_target_point_time = self.get_clock().now()
        self._last_depth_time = None
        self._last_tick_time = self.get_clock().now()
        self._last_cmd = (0.0, 0.0, 0.0, 0.0)

        self._xy_pid_x = PIDController(PIDGains(
            kp=float(self.get_parameter('staged_pid_xy_kp').value),
            ki=float(self.get_parameter('staged_pid_xy_ki').value),
            kd=float(self.get_parameter('staged_pid_xy_kd').value),
            integral_limit=float(self.get_parameter('staged_pid_xy_integral_limit').value),
            output_limit=self._max_vx,
        ))
        self._xy_pid_y = PIDController(PIDGains(
            kp=float(self.get_parameter('staged_pid_xy_kp').value),
            ki=float(self.get_parameter('staged_pid_xy_ki').value),
            kd=float(self.get_parameter('staged_pid_xy_kd').value),
            integral_limit=float(self.get_parameter('staged_pid_xy_integral_limit').value),
            output_limit=self._max_vy,
        ))
        self._z_pid = PIDController(PIDGains(
            kp=float(self.get_parameter('staged_pid_z_kp').value),
            ki=float(self.get_parameter('staged_pid_z_ki').value),
            kd=float(self.get_parameter('staged_pid_z_kd').value),
            integral_limit=float(self.get_parameter('staged_pid_z_integral_limit').value),
            output_limit=self._max_vz,
        ))
        self._staged_pid_enable_yaw = self._as_bool(self.get_parameter('staged_pid_enable_yaw').value)
        default_vertical_gate_tol_px = 150.0
        horizontal_tol_px = float(self.get_parameter('staged_pid_horizontal_tolerance_px').value)
        horizontal_tol_x_px = float(self.get_parameter('staged_pid_horizontal_tolerance_x_px').value)
        horizontal_tol_y_px = float(self.get_parameter('staged_pid_horizontal_tolerance_y_px').value)
        vertical_tol_px = float(self.get_parameter('staged_pid_vertical_tolerance_px').value)
        if horizontal_tol_x_px == default_vertical_gate_tol_px and horizontal_tol_px != default_vertical_gate_tol_px:
            horizontal_tol_x_px = horizontal_tol_px
        if horizontal_tol_y_px == default_vertical_gate_tol_px and horizontal_tol_px != default_vertical_gate_tol_px:
            horizontal_tol_y_px = horizontal_tol_px
        if horizontal_tol_y_px == default_vertical_gate_tol_px and vertical_tol_px != default_vertical_gate_tol_px:
            self.get_logger().warn(
                'staged_pid_vertical_tolerance_px is deprecated for vertical gate on/off; '
                'using it as a y-axis tolerance alias because staged_pid_horizontal_tolerance_y_px '
                'was left at its default.'
            )
            horizontal_tol_y_px = vertical_tol_px
        self._staged_pid_horizontal_tol_x_px = horizontal_tol_x_px
        self._staged_pid_horizontal_tol_y_px = horizontal_tol_y_px
        self._staged_pid_vertical_tol_px = horizontal_tol_y_px
        self._staged_pid_distance_tol_m = float(self.get_parameter('staged_pid_distance_tolerance_m').value)
        self._staged_pid_vertical_gate_window_ticks = max(
            1, int(self.get_parameter('staged_pid_vertical_gate_window_ticks').value)
        )
        self._staged_pid_vertical_enter_ratio = float(self.get_parameter('staged_pid_vertical_enter_ratio').value)
        self._staged_pid_vertical_exit_ratio = float(self.get_parameter('staged_pid_vertical_exit_ratio').value)
        self._staged_pid_vertical_enter_hold_ticks = max(
            1, int(self.get_parameter('staged_pid_vertical_enter_hold_ticks').value)
        )
        self._staged_pid_vertical_exit_hold_ticks = max(
            1, int(self.get_parameter('staged_pid_vertical_exit_hold_ticks').value)
        )
        self._staged_pid_hold_zero_distance_m = float(
            self.get_parameter('staged_pid_hold_zero_distance_m').value
        )
        self._tracked_enter_window_obs = max(
            1, int(self.get_parameter('tracked_enter_window_obs').value)
        )
        self._tracked_enter_min_obs = max(
            1, int(self.get_parameter('tracked_enter_min_obs').value)
        )
        self._tracked_exit_no_obs_sec = float(self.get_parameter('tracked_exit_no_obs_sec').value)
        self._target_tracked = False
        self._latest_detection_time = None
        self._last_valid_target_observation_time = None
        self._last_accepted_target_centroid: Optional[np.ndarray] = None
        self._tracking_valid_observation_history = deque(maxlen=self._tracked_enter_window_obs)
        self._staged_pid_state = self.StagedPIDState.NO_TARGET
        self._staged_pid_phase = 'horizontal_only'
        self._staged_pid_recent_horizontal_ok = deque(
            maxlen=self._staged_pid_vertical_gate_window_ticks
        )
        self._staged_pid_vertical_enabled = False
        self._staged_pid_enter_counter = 0
        self._staged_pid_exit_counter = 0
        self._staged_pid_horizontal_ok_ratio = 0.0
        self._staged_pid_vertical_scale = 0.0
        self._staged_pid_last_horizontal_aligned = False
        self._staged_pid_last_allow_z = False
        self._staged_pid_last_z_fresh = False
        self._staged_pid_last_z_meas: Optional[float] = None
        self._staged_pid_last_z_source = 'none'

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

        #self.target_centroid = np.mean(self.center_box_corners, axis=1)
        self.target_centroid = [cx, cy]
        self._staged_pid_target_centroid = np.array([self._target_u, self._target_v], dtype=np.float32)

        self.prev_centroid = np.zeros(2)
        self.roi_w = 50

        # define the minimum needed detections and the detection counter
        self.confirmation_time = 0.1  # Must see target for 0.1s to go VISUAL
        self.lost_timeout = 0.2        # Must lose target for 0.2s to go BREAK
        self._first_seen_time = None
        self.target_min = 1
        self.center_min = 1
        self.target_count = 0

        # IBVS parameters
        self.lmbda = 0          # the lambda control gain from the standard IBVS control law
        self.lmbda_max = self.ibvs_lmdba    # the maximum control gain that we can apply
        self.v_prev = np.zeros(6)
        self.alpha_1 = 0.6
        self.alpha_2 = 0.1
        self.v_last_valid = np.zeros(6)
        self.decay_rate = self.decay_rate_val  # Drops 5% of speed per "blind" frame
        self.DoF_mask = [1, 1, 0, 0, 0, 1]
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

        # data publishers
        self._cmd_pub = self.create_publisher(TwistStamped, self._cmd_topic, 10)                            # the actual commanded velocity in the Pixhawk base_link frame        
        self._sc2_cmd_pub_vec_6 = self.create_publisher(Vector6, self._sc2_cmd_topic_vec_6, 10)   
        
        # debug publishers 
        self._image_plane_vel_pub = self.create_publisher(TwistStamped, self._velocity_image_plane_frame_topic, 10)       # the velocities w.r.t. to the 
        self._sc2_base_link_vel_pub = self.create_publisher(TwistStamped, self._velocity_sc2_base_link_topic, 10)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._target_locked_pub = self.create_publisher(String, self._target_locked_topic, 10)
        self._centroid_error_publisher = self.create_publisher(Vector3, self._centroid_error_topic, 10)

        # subscribers
        self.create_subscription(Detection2D, self._target_bbox_topic, self._detection_cb, 10)
        self.create_subscription(CameraInfo, self._camera_info_topic, self._camera_info_cb, 10)
        self.create_subscription(TwistStamped, self._velocity_measurement_topic, self._vel_measurement_cb, 10)

        if self._controller_mode == 'staged_pid':
            self.create_subscription(PoseStamped, self._target_point_topic, self._target_pose_cb, 10)
            if self._depth_mode == 'DVL':
                self.create_subscription(
                    Dvl,
                    str(self.get_parameter('dvl_altitude_topic').value),
                    self._dvl_altitude_cb,
                    10,
                )
            elif self._depth_mode == 'MAVROS':
                self.create_subscription(
                    Float64,
                    str(self.get_parameter('mavros_altitude_topic').value),
                    self._mavros_altitude_cb,
                    10,
                )
        else:
            match self._depth_mode:
                case 'RGBD':
                    self.create_subscription(PoseStamped, self._target_pose_topic, self._target_pose_cb, 10)
                case 'DVL':
                    self.create_subscription(Dvl, self._target_pose_topic, self._target_pose_cb, 10)
                case 'MAVROS':
                    self.create_subscription(Float64, self._target_pose_topic, self._target_pose_cb, 10)

        
        #service for turning the IBVS control on or off 
        self.control_enabled = False # Default to off
        self.ibvs_switch_srv = self.create_service(SetBool, 'enable_ibvs_control', self.handle_control_switch)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        rate_hz = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / max(1e-3, rate_hz), self._tick)

        self.current_state = self.ControllerState.BREAK
        self._publish_status(str(self.current_state.name))

        self.get_logger().info(
            f'visual_servo_controller_node mode={self._controller_mode} target={self._target_bbox_topic} cmd={self._cmd_topic}'
        )

    def handle_control_switch(self, request, response):
        '''Service for turning the IBVS control on or off.'''
        self.control_enabled = request.data
        response.success = True
        response.message = f"IBVS Control {'Enabled' if self.control_enabled else 'Disabled'}"
        return response

    def _reset_staged_pid_state(self) -> None:
        self._xy_pid_x.reset()
        self._xy_pid_y.reset()
        self._reset_vertical_gate()

    def _reset_vertical_gate(self) -> None:
        self._z_pid.reset()
        self._staged_pid_recent_horizontal_ok.clear()
        self._staged_pid_vertical_enabled = False
        self._staged_pid_enter_counter = 0
        self._staged_pid_exit_counter = 0
        self._staged_pid_horizontal_ok_ratio = 0.0
        self._staged_pid_vertical_scale = 0.0
        self._staged_pid_last_horizontal_aligned = False
        self._staged_pid_last_allow_z = False
        self._staged_pid_last_z_fresh = False
        self._staged_pid_last_z_meas = None
        self._staged_pid_last_z_source = 'none'
        self._staged_pid_phase = 'horizontal_only'

    def _set_target_tracked(self, tracked: bool) -> None:
        if tracked == self._target_tracked:
            return
        self._target_tracked = tracked
        if tracked:
            self._staged_pid_state = self.StagedPIDState.HORIZONTAL
            self._reset_vertical_gate()
        else:
            self._tracking_valid_observation_history.clear()
            self._last_accepted_target_centroid = None
            self._staged_pid_state = self.StagedPIDState.NO_TARGET
            self._reset_staged_pid_state()

    def _is_timestamp_fresh(self, stamp, timeout_sec: float) -> bool:
        if stamp is None:
            return False
        return ((self.get_clock().now() - stamp).nanoseconds * 1e-9) <= timeout_sec

    def _update_target_tracking_from_detection(self, now) -> bool:
        if not hasattr(self, 'latest_centroid'):
            return False

        centroid = np.array(self.latest_centroid, dtype=np.float32)
        roi_consistent = (
            self._last_accepted_target_centroid is None or
            is_in_roi(centroid[0], centroid[1], self._last_accepted_target_centroid, self.roi_w)
        )
        valid_target_observation = bool(roi_consistent)
        self._tracking_valid_observation_history.append(valid_target_observation)

        if valid_target_observation:
            self._last_accepted_target_centroid = centroid
            self._last_valid_target_observation_time = now

        if (
            not self._target_tracked and
            sum(self._tracking_valid_observation_history) >= self._tracked_enter_min_obs
        ):
            self._set_target_tracked(True)

        return valid_target_observation

    def _update_target_tracked_from_tick(self) -> None:
        if (
            self._target_tracked and
            not self._is_timestamp_fresh(self._last_valid_target_observation_time, self._tracked_exit_no_obs_sec)
        ):
            self._set_target_tracked(False)

    def _tracked_horizontal_stale_command(self) -> tuple[float, float, float, float]:
        return (
            float(self.v_last_valid[0] * self.decay_rate),
            float(self.v_last_valid[1] * self.decay_rate),
            0.0,
            float(self.v_last_valid[5] * self.decay_rate),
        )

    def _compute_staged_pid_horizontal_command(self, dt_ctrl: float) -> tuple[float, float, float, float, str]:
        centroid_err_x = float(self.latest_centroid[0] - self._staged_pid_target_centroid[0])
        centroid_err_y = float(self.latest_centroid[1] - self._staged_pid_target_centroid[1])

        vel_x_im = self._xy_pid_x.update(centroid_err_x, dt_ctrl)
        vel_y_im = self._xy_pid_y.update(centroid_err_y, dt_ctrl)

        cmd_vx_body = vel_x_im
        cmd_vy_body = -vel_y_im
        yaw = 0.0
        if self._staged_pid_enable_yaw:
            yaw = clamp(
                0.8 * (centroid_err_x / max(1.0, self.image_width * 0.5)),
                self._max_yaw,
            )

        return cmd_vx_body, cmd_vy_body, 0.0, yaw, 'TRACKING_STAGED_PID_HORIZONTAL'

    def _update_staged_pid_vertical_gate(self, obs_fresh: bool) -> tuple[bool, float]:
        horizontal_aligned = False
        if obs_fresh and hasattr(self, 'latest_centroid'):
            centroid_err_x = float(self.latest_centroid[0] - self._staged_pid_target_centroid[0])
            centroid_err_y = float(self.latest_centroid[1] - self._staged_pid_target_centroid[1])
            horizontal_aligned = (
                abs(centroid_err_x) <= self._staged_pid_horizontal_tol_x_px and
                abs(centroid_err_y) <= self._staged_pid_horizontal_tol_y_px
            )

        self._staged_pid_recent_horizontal_ok.append(horizontal_aligned)
        ok_count = sum(self._staged_pid_recent_horizontal_ok)
        horizontal_ok_ratio = ok_count / float(self._staged_pid_vertical_gate_window_ticks)

        if horizontal_ok_ratio >= self._staged_pid_vertical_enter_ratio:
            self._staged_pid_enter_counter += 1
        else:
            self._staged_pid_enter_counter = 0

        if horizontal_ok_ratio <= self._staged_pid_vertical_exit_ratio:
            self._staged_pid_exit_counter += 1
        else:
            self._staged_pid_exit_counter = 0

        if (
            not self._staged_pid_vertical_enabled and
            self._staged_pid_enter_counter >= self._staged_pid_vertical_enter_hold_ticks
        ):
            self._staged_pid_vertical_enabled = True
            self._staged_pid_exit_counter = 0
        elif (
            self._staged_pid_vertical_enabled and
            self._staged_pid_exit_counter >= self._staged_pid_vertical_exit_hold_ticks
        ):
            self._staged_pid_vertical_enabled = False
            self._staged_pid_enter_counter = 0
            self._z_pid.reset()

        self._staged_pid_phase = (
            'vertical_enabled' if self._staged_pid_vertical_enabled else 'horizontal_only'
        )
        self._staged_pid_horizontal_ok_ratio = horizontal_ok_ratio
        self._staged_pid_vertical_scale = horizontal_ok_ratio if self._staged_pid_vertical_enabled else 0.0
        self._staged_pid_last_horizontal_aligned = horizontal_aligned
        return horizontal_aligned, horizontal_ok_ratio

    def _is_staged_pid_vertical_gate_aligned(self) -> bool:
        if not hasattr(self, 'latest_centroid'):
            return False
        centroid_err_x = float(self.latest_centroid[0] - self._staged_pid_target_centroid[0])
        centroid_err_y = float(self.latest_centroid[1] - self._staged_pid_target_centroid[1])
        return (
            abs(centroid_err_x) <= self._staged_pid_horizontal_tol_x_px and
            abs(centroid_err_y) <= self._staged_pid_horizontal_tol_y_px
        )

    def _resolve_staged_pid_vertical_measurement(self) -> tuple[Optional[float], str, bool]:
        target_point_fresh = (
            self._latest_target_point is not None and
            self._is_timestamp_fresh(self._last_target_point_time, self._target_point_timeout_sec)
        )
        dvl_fresh = (
            self._latest_depth_measurement is not None and
            self._is_timestamp_fresh(self._last_depth_time, self._target_point_timeout_sec)
        )

        if self._staged_pid_z_source in {'target_point', 'target_point_primary_dvl_fallback'}:
            if target_point_fresh:
                return float(self._latest_target_point.pose.position.z), 'target_point', True
            if self._staged_pid_z_source == 'target_point_primary_dvl_fallback' and dvl_fresh:
                return float(self._latest_depth_measurement), 'dvl_fallback', True
            return None, 'none', False

        if self._staged_pid_z_source == 'dvl':
            if dvl_fresh:
                return float(self._latest_depth_measurement), 'dvl', True
            return None, 'none', False

        return None, 'none', False

    def _compute_staged_pid_command(
        self,
        obs_fresh: bool,
        dt_ctrl: float,
    ) -> tuple[float, float, float, float, str, bool, Optional[float], bool, str]:
        vx_body, vy_body, _, yaw, _ = self._compute_staged_pid_horizontal_command(dt_ctrl)
        horizontal_aligned_now, _ = self._update_staged_pid_vertical_gate(obs_fresh)
        z_meas, z_source, z_fresh = self._resolve_staged_pid_vertical_measurement()
        allow_z = (
            self._staged_pid_vertical_enabled and
            horizontal_aligned_now and
            z_fresh and
            z_meas is not None
        )

        vz_body = 0.0
        if allow_z:
            err_z_m = float(z_meas - self._desired_vertical_dist)
            vz_raw = self._z_pid.update(err_z_m, dt_ctrl)
            vz_body = self._staged_pid_vertical_scale * vz_raw
            aligned = (
                self._is_staged_pid_vertical_gate_aligned() and
                abs(err_z_m) <= self._staged_pid_distance_tol_m
            )
            # Phase 2 only exposes the target-distance terminal condition as
            # status. It does not enter a hold-zero state or change the command.
            if z_meas <= self._staged_pid_hold_zero_distance_m:
                status = f'TRACKING_STAGED_PID_TARGET_DISTANCE_REACHED_{z_source.upper()}'
            else:
                source_suffix = z_source.upper()
                status = (
                    f'TRACKING_STAGED_PID_ALIGNED_{source_suffix}'
                    if aligned else
                    f'TRACKING_STAGED_PID_VERTICAL_{source_suffix}'
                )
            self._staged_pid_state = self.StagedPIDState.HORIZONTAL_AND_VERTICAL
        else:
            self._z_pid.reset()
            if self._staged_pid_vertical_enabled and horizontal_aligned_now and not z_fresh:
                status = 'TRACKING_STAGED_PID_HORIZONTAL_Z_STALE'
            elif horizontal_aligned_now:
                status = 'TRACKING_STAGED_PID_HORIZONTAL_READY'
            else:
                status = 'TRACKING_STAGED_PID_HORIZONTAL'
            self._staged_pid_state = self.StagedPIDState.HORIZONTAL

        self._staged_pid_last_allow_z = allow_z
        self._staged_pid_last_z_fresh = z_fresh
        self._staged_pid_last_z_meas = z_meas
        self._staged_pid_last_z_source = z_source
        return vx_body, vy_body, vz_body, yaw, status, allow_z, z_meas, z_fresh, z_source

    def _target_pose_cb(self, msg) -> None:
        '''Extract the depth information from the latest Pose estimate.'''

        if self._controller_mode == 'staged_pid':
            self._latest_target_point = msg
            self._last_target_point_time = self.get_clock().now()
            return

        match self._depth_mode:
            case 'RGBD':
                latest_pose = msg.pose
                self._latest_depth_measurement = latest_pose.position.z
            case 'DVL':
                latest_depth = msg.altitude # get the altitude reading from the velocity report
                self._latest_depth_measurement = np.abs(latest_depth)   # take the absoloute value, since depth w.r.t. the camera can only be positive
            case 'MAVROS':
                latest_depth = msg.data
                self._latest_depth_measurement = np.abs(latest_depth)

    def _dvl_altitude_cb(self, msg: Dvl) -> None:
        # Phase 2 staged PID fallback. target_point remains primary while fresh.
        self._latest_depth_measurement = float(np.abs(msg.altitude))
        self._last_depth_time = self.get_clock().now()

    def _mavros_altitude_cb(self, msg: Float64) -> None:
        # Compatibility hook. The default staged PID fallback uses DVL.
        self._latest_depth_measurement = float(np.abs(msg.data))
        self._last_depth_time = self.get_clock().now()

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

        now = self.get_clock().now()
        self._last_obs_time = now
        self._latest_detection_time = now
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

        #self.latest_centroid = np.mean(self.latest_bbox_corners, axis=1).copy()
        self.latest_centroid = [best_bbox.center.position.x ,best_bbox.center.position.y]

        if self._controller_mode == 'staged_pid':
            centroid_err_x = self.latest_centroid[0] - self._staged_pid_target_centroid[0]
            centroid_err_y = self.latest_centroid[1] - self._staged_pid_target_centroid[1]
        else:
            centroid_err_x = self.target_centroid[0] - self.latest_centroid[0]
            centroid_err_y = self.target_centroid[1] - self.latest_centroid[1]

        err_summary = Vector3()
        err_summary.x = float(centroid_err_x)
        err_summary.y = float(centroid_err_y)
        err_summary.z = float(self._latest_depth_measurement - self._desired_target_dist)
        self._centroid_error_publisher.publish(err_summary)

        if self._controller_mode == 'staged_pid':
            self._update_target_tracking_from_detection(now)

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt_obs = (now - self._last_obs_time).nanoseconds * 1e-9
        dt_ctrl = max(1e-4, (now - self._last_tick_time).nanoseconds * 1e-9)
        self._last_tick_time = now

        if not self.control_enabled:
            if self._controller_mode == 'staged_pid':
                self._reset_staged_pid_state()
            self._publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.get_logger().info('IBVS Control is off...use service call to turn it on: ros2 service call /enable_ibvs_control std_srvs/srv/SetBool "{data: true}"', throttle_duration_sec=2.0)
            return

        if self._controller_mode == 'staged_pid':
            self._tick_staged_pid(dt_ctrl, dt_obs)
            return

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

        # --- 2. MATH UPDATE (Only if Locked) ---
        if is_locked:
            self.current_e = (self.center_box_corners - self.latest_bbox_corners).flatten(order="F") 
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
                #v_raw_target = (-self.lmbda_max * inv_J @ self.current_e) * self.DoF_mask
                #NOTE: removed negative sign, according to Corke: Robotics, Vision and Control, must correspond to correct error computation order
                v_raw_target = (self.lmbda_max * inv_J @ self.current_e) * self.DoF_mask    
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

        # map the velocities computed in the image plane 
        v_image_plane_msg = TwistStamped()
        v_image_plane_msg.header.stamp = self.get_clock().now().to_msg()
        v_image_plane_msg.header.frame_id = self._camera_image_plane_frame_id
        
        v_image_plane_msg.twist.linear.x = vx    
        v_image_plane_msg.twist.linear.y = vy    
        v_image_plane_msg.twist.linear.z = vz   
        v_image_plane_msg.twist.angular.z = yaw  

        # PUBLISH the camera frame debug velocity
        self._image_plane_vel_pub.publish(v_image_plane_msg)

        # 2. Prepare for TF Transformation (Linear and Angular separately)
        v_lin_image_plane = Vector3Stamped()
        v_lin_image_plane.header = v_image_plane_msg.header
        v_lin_image_plane.vector = v_image_plane_msg.twist.linear

        v_ang_image_plane = Vector3Stamped()
        v_ang_image_plane.header = v_image_plane_msg.header
        v_ang_image_plane.vector = v_image_plane_msg.twist.angular

        try:
            # Look up the transform from Optical Frame to ROV Body Frame
            transform = self.tf_buffer.lookup_transform(
                self._sc2_base_link_frame_id, 
                self._camera_image_plane_frame_id, 
                rclpy.time.Time()
            )
            
            # Transform both vectors to the ROV's coordinate system
            v_lin_sc2_base_link = tf2_geometry_msgs.do_transform_vector3(v_lin_image_plane, transform)
            v_ang_sc2_base_link = tf2_geometry_msgs.do_transform_vector3(v_ang_image_plane, transform)

            # 3. Publish to BlueROV2 (MAVROS)
            # We take the resulting Z-axis rotation as the ROV's Yaw
            if self._yaw_control_switch:
                yaw_cmd = v_ang_sc2_base_link.vector.z
            else:
                yaw_cmd = 0.0 

            self._publish_cmd_with_slew(
                v_lin_sc2_base_link.vector.x, 
                v_lin_sc2_base_link.vector.y, 
                v_lin_sc2_base_link.vector.z, 
                yaw_cmd,
                dt_ctrl
            )

            # publish the velocities in the reference sc2_base_link frame for debugging    
            v_sc2_base_link_msg = TwistStamped()
            v_sc2_base_link_msg.header = v_image_plane_msg.header
            v_sc2_base_link_msg.header.frame_id = self._sc2_base_link_frame_id
            v_sc2_base_link_msg.twist.linear.x = v_lin_sc2_base_link.vector.x
            v_sc2_base_link_msg.twist.linear.y = v_lin_sc2_base_link.vector.y
            v_sc2_base_link_msg.twist.linear.z = v_lin_sc2_base_link.vector.z
            v_sc2_base_link_msg.twist.angular.z = v_ang_sc2_base_link.vector.z

            self._sc2_base_link_vel_pub.publish(v_sc2_base_link_msg)

        except Exception as e:
            self.get_logger().error(f"TF Transform failed: {e}", throttle_duration_sec=1.0)

    def _tick_staged_pid(self, dt_ctrl: float, dt_obs: float) -> None:
        obs_fresh = self._latest_obs is not None and dt_obs <= self._timeout_sec
        self._update_target_tracked_from_tick()
        self._publish_is_locked(str(self._target_tracked))

        if not self._target_tracked:
            self.set_state(self.ControllerState.BREAK)
            self.v_last_valid = np.zeros(6)
            self._publish_status('TRACKING_STAGED_PID_NO_TARGET')
            self._publish_cmd_direct_body_with_slew(0.0, 0.0, 0.0, 0.0, dt_ctrl)
            return

        self.set_state(self.ControllerState.VISUAL)

        if obs_fresh and hasattr(self, 'latest_centroid'):
            vx_body, vy_body, vz_body, yaw, status, *_ = self._compute_staged_pid_command(
                obs_fresh,
                dt_ctrl,
            )
        else:
            self._staged_pid_state = self.StagedPIDState.HORIZONTAL
            self._reset_vertical_gate()
            vx_body, vy_body, vz_body, yaw = self._tracked_horizontal_stale_command()
            status = 'TRACKING_STAGED_PID_HORIZONTAL_STALE'

        self.v_last_valid = np.array([vx_body, vy_body, vz_body, 0.0, 0.0, yaw])
        self._publish_status(status)
        self._publish_cmd_direct_body_with_slew(vx_body, vy_body, vz_body, yaw, dt_ctrl)

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
        cmd.header.frame_id = self._mavros_base_link_frame_id
        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        cmd.twist.linear.z = vz
        cmd.twist.angular.z = yaw

        self._cmd_pub.publish(cmd)
        self._last_cmd = (vx, vy, vz, yaw)

        # publish the topic for SC2 as a custom Vector6 type, defined in the state_machine package
        sc2_cmd_linear = Vector3()
        sc2_cmd_linear.x = vx
        sc2_cmd_linear.y = vy
        sc2_cmd_linear.z = vz

        sc2_cmd_angular = Vector3()
        sc2_cmd_angular.x = 0.0 # assuiming no roll control
        sc2_cmd_angular.y = 0.0 # assuming no pitch control
        sc2_cmd_angular.z = yaw

        cmd_vec_6 = Vector6()
        cmd_vec_6.linear = sc2_cmd_linear
        cmd_vec_6.angular = sc2_cmd_angular   
        self._sc2_cmd_pub_vec_6.publish(cmd_vec_6)

    def _publish_cmd_direct_body(self, vx: float, vy: float, vz: float, yaw: float) -> None:
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

        sc2_cmd_linear = Vector3()
        sc2_cmd_linear.x = vx
        sc2_cmd_linear.y = vy
        sc2_cmd_linear.z = vz

        sc2_cmd_angular = Vector3()
        sc2_cmd_angular.x = 0.0
        sc2_cmd_angular.y = 0.0
        sc2_cmd_angular.z = yaw

        cmd_vec_6 = Vector6()
        cmd_vec_6.linear = sc2_cmd_linear
        cmd_vec_6.angular = sc2_cmd_angular
        self._sc2_cmd_pub_vec_6.publish(cmd_vec_6)

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
        
        transform = self.tf_buffer.lookup_transform(
                self._mavros_base_link_frame_id,              # pixhawk base_link frame
                self._sc2_base_link_frame_id,                    # our base_link frame
                rclpy.time.Time() 
            )

        vel_sc2_base_link = PointStamped()
        vel_sc2_base_link.point.x = vx
        vel_sc2_base_link.point.y = vy
        vel_sc2_base_link.point.z = vz

        vel_mavros_base_link = tf2_geometry_msgs.do_transform_point(vel_sc2_base_link, transform)

        vx_mavros_base_link = vel_mavros_base_link.point.x
        vy_mavros_base_link = vel_mavros_base_link.point.y
        vz_mavros_base_link = vel_mavros_base_link.point.z

        # NOTE: yaw stays the same in both frames

        pvx, pvy, pvz, pyaw = self._last_cmd
        svx = self._slew_axis(vx_mavros_base_link, pvx, self._slew_vx, dt)
        svy = self._slew_axis(vy_mavros_base_link, pvy, self._slew_vy, dt)
        svz = self._slew_axis(vz_mavros_base_link, pvz, self._slew_vz, dt)
        syaw = self._slew_axis(yaw, pyaw, self._slew_yaw, dt)
        self._publish_cmd(svx, svy, svz, syaw)

    def _publish_cmd_direct_body_with_slew(self, vx: float, vy: float, vz: float, yaw: float, dt: float) -> None:
        pvx, pvy, pvz, pyaw = self._last_cmd
        svx = self._slew_axis(vx, pvx, self._slew_vx, dt)
        svy = self._slew_axis(vy, pvy, self._slew_vy, dt)
        svz = self._slew_axis(vz, pvz, self._slew_vz, dt)
        syaw = self._slew_axis(yaw, pyaw, self._slew_yaw, dt)
        self._publish_cmd_direct_body(svx, svy, svz, syaw)

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
