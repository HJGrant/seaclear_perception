import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sc2_visual_servo_interfaces.msg import TargetObservation
from state_estimation.msg import DVL, Vector6
from std_msgs.msg import String
from std_srvs.srv import SetBool
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


OBS_LOG = "/tmp/target_observer_node.log"
CTRL_LOG = "/tmp/visual_servo_controller_node.log"


OBSERVER_CMD = [
    "ros2",
    "run",
    "sc2_visual_servo_perception",
    "target_observer_node",
    "--ros-args",
    "-p",
    "detections_topic:=/detections_output",
    "-p",
    "output_topic:=/sc2/visual_servo/target_obs",
    "-p",
    'target_class:="3"',
    "-p",
    "min_confidence:=0.5",
    "-p",
    "low_confidence:=0.15",
    "-p",
    "n_confirm:=3",
    "-p",
    "t_stale_sec:=1.0",
    "-p",
    "timeout_ms:=300",
    "-p",
    "use_depth:=false",
    "-p",
    "base_gate_px:=80.0",
    "-p",
    "selector_switch_margin:=0.05",
    "-p",
    "selector_selected_track_bonus:=0.05",
    "-p",
    "enable_motion_aware_gate:=false",
]


CONTROLLER_CMD = [
    "ros2",
    "run",
    "sc2_visual_servo_controller",
    "visual_servo_controller_node",
    "--ros-args",
    "-p",
    "use_target_observation:=true",
    "-p",
    "target_topic:=/sc2/visual_servo/target_obs",
    "-p",
    "controller_mode:=staged_pid",
    "-p",
    "depth_estimation_mode:=DVL",
    "-p",
    "staged_pid_z_source:=dvl",
    "-p",
    "dvl_altitude_topic:=/waterlinked_dvl_driver/velocity_report",
    "-p",
    "timeout_ms:=200",
    "-p",
    "target_point_timeout_ms:=200",
    "-p",
    "rate_hz:=10.0",
    "-p",
    "image_width:=640.0",
    "-p",
    "image_height:=640.0",
    "-p",
    "target_u:=320.0",
    "-p",
    "target_v:=320.0",
    "-p",
    "desired_vertical_distance_m:=1.0",
    "-p",
    "hold_maintain_enable:=true",
    "-p",
    "hold_maintain_velocity_damping_enable:=true",
    "-p",
    "hold_maintain_xy_damping:=0.4",
    "-p",
    "hold_maintain_yaw_damping:=0.3",
    "-p",
    "hold_maintain_velocity_timeout_ms:=500",
    "-p",
    "cmd_topic_vec_6:=/sc2/velocity_controller_node/outer/console_signals",
]


class Harness(Node):
    def __init__(self):
        super().__init__("vs_audit_harness")
        self.det_pub = self.create_publisher(Detection2DArray, "/detections_output", 10)
        self.vel_pub = self.create_publisher(TwistStamped, "/mavros/local_position/velocity_body", 10)
        self.dvl_pub = self.create_publisher(DVL, "/waterlinked_dvl_driver/velocity_report", 10)
        self.obs_msgs = []
        self.status_msgs = []
        self.cmd_msgs = []
        self.create_subscription(TargetObservation, "/sc2/visual_servo/target_obs", self._obs_cb, 10)
        self.create_subscription(String, "/sc2/visual_servo/status", self._status_cb, 10)
        self.create_subscription(Vector6, "/sc2/velocity_controller_node/outer/console_signals", self._cmd_cb, 10)
        self.client = self.create_client(SetBool, "/enable_ibvs_control")

    def _obs_cb(self, msg):
        self.obs_msgs.append(
            {
                "t": time.time(),
                "selected_valid": bool(msg.selected_valid),
                "selected_track_id": int(msg.selected_track_id),
                "track_state": msg.track_state,
                "reason_code": msg.reason_code,
                "u": float(msg.u),
                "v": float(msg.v),
                "freshness": float(msg.freshness),
            }
        )

    def _status_cb(self, msg):
        self.status_msgs.append({"t": time.time(), "status": msg.data})

    def _cmd_cb(self, msg):
        self.cmd_msgs.append(
            {
                "t": time.time(),
                "vx": float(msg.linear.x),
                "vy": float(msg.linear.y),
                "vz": float(msg.linear.z),
                "yaw": float(msg.angular.z),
            }
        )

    def publish_supporting_msgs(self):
        stamp = self.get_clock().now().to_msg()

        vel = TwistStamped()
        vel.header.stamp = stamp
        vel.header.frame_id = "base_link"
        vel.twist.linear.x = 0.2
        vel.twist.linear.y = -0.1
        vel.twist.angular.z = 0.15
        self.vel_pub.publish(vel)

        dvl = DVL()
        dvl.header.stamp = stamp
        dvl.header.frame_id = "base_link"
        dvl.altitude = 1.3
        self.dvl_pub.publish(dvl)

    def publish_pair(self, score_a, score_b):
        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "oak_rgb_camera_optical_frame"
        msg.detections.append(self._det(260.0, 300.0, 90.0, 70.0, score_a))
        msg.detections.append(self._det(420.0, 300.0, 90.0, 70.0, score_b))
        self.det_pub.publish(msg)

    @staticmethod
    def _det(u, v, w, h, score):
        det = Detection2D()
        det.bbox.center.position.x = float(u)
        det.bbox.center.position.y = float(v)
        det.bbox.size_x = float(w)
        det.bbox.size_y = float(h)
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = "3"
        hyp.hypothesis.score = float(score)
        det.results.append(hyp)
        return det


def _tail(path: str, n: int = 2000) -> str:
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = fh.read()
        return data[-n:]
    except FileNotFoundError:
        return ""


def main():
    obs_log = open(OBS_LOG, "w", encoding="utf-8")
    ctrl_log = open(CTRL_LOG, "w", encoding="utf-8")
    observer = subprocess.Popen(OBSERVER_CMD, stdout=obs_log, stderr=subprocess.STDOUT, text=True)
    controller = subprocess.Popen(CONTROLLER_CMD, stdout=ctrl_log, stderr=subprocess.STDOUT, text=True)

    rclpy.init()
    node = Harness()

    try:
        deadline = time.time() + 15.0
        while time.time() < deadline:
            if node.client.wait_for_service(timeout_sec=0.2):
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        else:
            raise RuntimeError("enable_ibvs_control service did not appear")

        req = SetBool.Request()
        req.data = True
        future = node.client.call_async(req)
        service_deadline = time.time() + 5.0
        while time.time() < service_deadline and not future.done():
            node.publish_supporting_msgs()
            rclpy.spin_once(node, timeout_sec=0.1)
        if not future.done() or not future.result().success:
            raise RuntimeError("failed to enable controller")

        t0 = time.time()
        end = t0 + 5.5
        while time.time() < end:
            elapsed = time.time() - t0
            node.publish_supporting_msgs()
            if elapsed < 1.5:
                node.publish_pair(0.85, 0.80)
            elif elapsed < 3.0:
                node.publish_pair(0.84, 0.89)
            elif elapsed < 4.0:
                node.publish_pair(0.74, 0.92)
            rclpy.spin_once(node, timeout_sec=0.1)

        valid_obs = [m for m in node.obs_msgs if m["selected_valid"]]
        hold_obs = [m for m in node.obs_msgs if (not m["selected_valid"]) and m["track_state"] == "HOLD"]
        hold_status = [m for m in node.status_msgs if "HOLD_SELECTED_INVALID" in m["status"]]
        phase1 = [m for m in valid_obs if m["t"] - t0 < 1.6]
        phase2 = [m for m in valid_obs if 1.8 <= m["t"] - t0 < 2.8]
        phase3 = [m for m in valid_obs if 3.1 <= m["t"] - t0 < 3.9]
        phase1_ids = sorted({m["selected_track_id"] for m in phase1})
        phase2_ids = sorted({m["selected_track_id"] for m in phase2})
        phase3_ids = sorted({m["selected_track_id"] for m in phase3})
        first_hold = hold_obs[0] if hold_obs else None
        hold_status_runtime = []
        hold_cmds = []
        if first_hold is not None:
            hold_status_runtime = [
                m
                for m in node.status_msgs
                if "HOLD_SELECTED_INVALID" in m["status"] and m["t"] >= first_hold["t"] - 0.05
            ]
            hold_cmds = [
                m
                for m in node.cmd_msgs
                if m["t"] >= first_hold["t"] and (
                    abs(m["vx"]) > 1e-6 or
                    abs(m["vy"]) > 1e-6 or
                    abs(m["vz"]) > 1e-6 or
                    abs(m["yaw"]) > 1e-6
                )
            ]

        print("RUNTIME_SUMMARY")
        print(
            "obs_count={} valid_obs_count={} hold_obs_count={}".format(
                len(node.obs_msgs), len(valid_obs), len(hold_obs)
            )
        )
        print("phase1_ids={}".format(phase1_ids))
        print("phase2_ids={}".format(phase2_ids))
        print("phase3_ids={}".format(phase3_ids))
        if valid_obs:
            print("first_valid={}".format(valid_obs[0]))
            print("last_valid={}".format(valid_obs[-1]))
        if first_hold is not None:
            print("first_hold={}".format(first_hold))
        if hold_status_runtime:
            print("first_runtime_hold_status={}".format(hold_status_runtime[0]["status"]))
        if hold_cmds:
            print("first_nonzero_hold_cmd={}".format(hold_cmds[0]))

    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()
        for proc in (controller, observer):
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=5.0)
        obs_log.close()
        ctrl_log.close()
        print("LOGFILE={}".format(OBS_LOG))
        print(_tail(OBS_LOG))
        print("LOGFILE={}".format(CTRL_LOG))
        print(_tail(CTRL_LOG))


if __name__ == "__main__":
    main()
