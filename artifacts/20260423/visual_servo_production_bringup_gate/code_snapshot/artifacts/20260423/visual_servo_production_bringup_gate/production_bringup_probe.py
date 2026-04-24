import signal
import subprocess
import time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sc2_visual_servo_interfaces.msg import TargetObservation
from state_estimation.msg import Vector6
from std_msgs.msg import String
from std_srvs.srv import SetBool


LAUNCH_LOG = "/tmp/object_detection_ibvs_launch.log"
BAG_LOG = "/tmp/object_detection_ibvs_bag_play.log"
BAG_PATH = "/ws/rosbag2_2026_04_22-16_47_57-diving-noimage"


LAUNCH_CMD = [
    "ros2",
    "launch",
    "sc2_visual_servo_bringup",
    "object_detection_ibvs.launch.py",
    "launch_foxglove:=false",
    "use_sim_time:=true",
]


BAG_CMD = [
    "ros2",
    "bag",
    "play",
    BAG_PATH,
    "--topics",
    "/detections_output",
    "/sc2/imu/imu",
    "/sc2/odometry_ekf",
    "/sc2/dvl/dvl",
    "/waterlinked_dvl_driver/velocity_report",
    "--clock",
]


class Probe(Node):
    def __init__(self):
        super().__init__("production_bringup_probe")
        self.client = self.create_client(SetBool, "/enable_ibvs_control")
        self.obs_msgs = []
        self.status_msgs = []
        self.lock_msgs = []
        self.cmd_msgs = []
        self.vel_bridge_pub = self.create_publisher(
            TwistStamped,
            "/mavros/local_position/velocity_body",
            10,
        )
        self.create_subscription(TargetObservation, "/sc2/visual_servo/target_obs", self._obs_cb, 50)
        self.create_subscription(String, "/sc2/visual_servo/status", self._status_cb, 50)
        self.create_subscription(String, "/sc2/visual_servo/target_locked", self._lock_cb, 50)
        self.create_subscription(Vector6, "/sc2/velocity_controller_node/outer/console_signals", self._cmd_cb, 50)
        self.create_subscription(Odometry, "/sc2/dvl/dvl", self._odom_cb, 50)

    def _obs_cb(self, msg):
        self.obs_msgs.append(
            {
                "t": time.time(),
                "selected_valid": bool(msg.selected_valid),
                "selected_track_id": int(msg.selected_track_id),
                "track_state": str(msg.track_state),
                "reason_code": str(msg.reason_code),
                "freshness": float(msg.freshness),
                "u": float(msg.u),
                "v": float(msg.v),
            }
        )

    def _status_cb(self, msg):
        self.status_msgs.append({"t": time.time(), "status": str(msg.data)})

    def _lock_cb(self, msg):
        self.lock_msgs.append({"t": time.time(), "value": str(msg.data)})

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

    def _odom_cb(self, msg: Odometry):
        out = TwistStamped()
        out.header = msg.header
        out.header.frame_id = "base_link"
        out.twist = msg.twist.twist
        self.vel_bridge_pub.publish(out)


def _tail(path: str, size: int = 3000) -> str:
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = fh.read()
        return data[-size:]
    except FileNotFoundError:
        return ""


def _shutdown_process(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5.0)


def main():
    launch_log = open(LAUNCH_LOG, "w", encoding="utf-8")
    bag_log = open(BAG_LOG, "w", encoding="utf-8")
    launch_proc = subprocess.Popen(LAUNCH_CMD, stdout=launch_log, stderr=subprocess.STDOUT, text=True)
    bag_proc = None

    rclpy.init()
    node = Probe()

    try:
        deadline = time.time() + 20.0
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
            rclpy.spin_once(node, timeout_sec=0.1)
        if not future.done() or not future.result().success:
            raise RuntimeError("failed to enable controller")

        bag_proc = subprocess.Popen(BAG_CMD, stdout=bag_log, stderr=subprocess.STDOUT, text=True)
        while bag_proc.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.1)

        post_deadline = time.time() + 2.5
        while time.time() < post_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        valid_obs = [m for m in node.obs_msgs if m["selected_valid"]]
        invalid_obs = [m for m in node.obs_msgs if not m["selected_valid"]]
        hold_obs = [m for m in invalid_obs if m["track_state"] == "HOLD"]
        hold_statuses = [m for m in node.status_msgs if "HOLD_SELECTED_INVALID" in m["status"]]

        transitions = []
        prev = None
        for msg in valid_obs:
            tid = msg["selected_track_id"]
            if prev is None:
                prev = tid
                continue
            if tid != prev:
                transitions.append({"t": msg["t"], "from": prev, "to": tid})
                prev = tid

        nonzero_hold_cmd = None
        if hold_obs:
            hold_t = hold_obs[0]["t"]
            for cmd in node.cmd_msgs:
                if cmd["t"] < hold_t:
                    continue
                if any(abs(cmd[k]) > 1e-6 for k in ("vx", "vy", "vz", "yaw")):
                    nonzero_hold_cmd = cmd
                    break

        print("PRODUCTION_BRINGUP_SUMMARY")
        print("obs_count={}".format(len(node.obs_msgs)))
        print("valid_obs_count={}".format(len(valid_obs)))
        print("invalid_obs_count={}".format(len(invalid_obs)))
        print("hold_obs_count={}".format(len(hold_obs)))
        print("status_count={}".format(len(node.status_msgs)))
        print("target_locked_count={}".format(len(node.lock_msgs)))
        print("cmd_count={}".format(len(node.cmd_msgs)))
        print("valid_track_ids={}".format(sorted({m["selected_track_id"] for m in valid_obs})))
        print("transition_count={}".format(len(transitions)))
        if transitions:
            print("first_transition={}".format(transitions[0]))
        if valid_obs:
            print("first_valid={}".format(valid_obs[0]))
            print("last_valid={}".format(valid_obs[-1]))
        if hold_obs:
            print("first_hold={}".format(hold_obs[0]))
        if hold_statuses:
            first_hold_status = hold_statuses[0]["status"]
            if hold_obs:
                hold_t = hold_obs[0]["t"]
                for status in hold_statuses:
                    if status["t"] >= hold_t:
                        first_hold_status = status["status"]
                        break
            print("first_hold_status={}".format(first_hold_status))
        if nonzero_hold_cmd is not None:
            print("first_nonzero_hold_cmd={}".format(nonzero_hold_cmd))

    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()
        if bag_proc is not None:
            _shutdown_process(bag_proc)
        _shutdown_process(launch_proc)
        launch_log.close()
        bag_log.close()
        print("LOGFILE={}".format(LAUNCH_LOG))
        print(_tail(LAUNCH_LOG))
        print("LOGFILE={}".format(BAG_LOG))
        print(_tail(BAG_LOG))


if __name__ == "__main__":
    main()
