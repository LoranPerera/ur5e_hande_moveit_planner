#!/usr/bin/env python3
"""
Fast Robotiq Hand-E Bridge
--------------------------
Simplified for binary open/close control.

MoveIt joint values:
  open  = 0.025
  close = 0.0

Real gripper POS:
  open  = 3
  close = 248
"""

import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String


# ---------------------------------------------------------------------------
# Constants — discrete open/close
# ---------------------------------------------------------------------------
JOINT_NAME      = "robotiq_hande_left_finger_joint"

# MoveIt joint values
JOINT_OPEN      = 0.025
JOINT_CLOSED    = 0.0

# Real gripper POS values
POS_OPEN        = 3
POS_CLOSED      = 248

DEFAULT_SPEED   = 255
DEFAULT_FORCE   = 150
RECONNECT_DELAY = 2.0


class RobotiqHandEBridge(Node):

    def __init__(self):
        super().__init__("robotiq_hande_bridge")

        # Parameters
        self.declare_parameter("gripper_ip",         "192.168.1.100")
        self.declare_parameter("gripper_port",       63352)
        self.declare_parameter("joint_state_rate",   20.0)
        self.declare_parameter("action_server_name",
                               "/hand_e_controller/follow_joint_trajectory")

        self._ip          = self.get_parameter("gripper_ip").value
        self._port        = self.get_parameter("gripper_port").value
        self._rate        = self.get_parameter("joint_state_rate").value
        self._action_name = self.get_parameter("action_server_name").value

        # Socket state
        self._sock: socket.socket | None = None
        self._sock_lock   = threading.Lock()
        self._current_pos: int = POS_OPEN
        self._connected   = False

        # Publishers
        self._js_pub    = self.create_publisher(JointState, "/joint_states",        10)
        self._bool_pub  = self.create_publisher(Bool,       "/gripper/is_closed",   10)
        self._named_pub = self.create_publisher(String,     "/gripper/named_state", 10)

        # Timer for publishing state
        self.create_timer(1.0 / self._rate, self._publish_state)

        # Action server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self._action_name,
            goal_callback    = self._goal_callback,
            cancel_callback  = self._cancel_callback,
            execute_callback = self._execute_callback,
        )

        # Connect to gripper
        self._connect()

        self.get_logger().info(
            f"Fast Hand-E bridge ready ({self._ip}:{self._port})\n"
            f"Listening for joint: {JOINT_NAME}"
        )

    # ----------------------------------------------------------------------
    # Socket
    # ----------------------------------------------------------------------
    def _connect(self):
        while rclpy.ok():
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((self._ip, self._port))
                with self._sock_lock:
                    self._sock      = sock
                    self._connected = True

                self._send_raw(f"SET SPE {DEFAULT_SPEED}")
                self._send_raw(f"SET FOR {DEFAULT_FORCE}")

                pos_str = self._send_raw("GET POS")
                self._current_pos = int(pos_str.strip()) if pos_str.isdigit() else POS_OPEN

                self.get_logger().info("Connected to Robotiq Hand-E gripper.")
                return

            except Exception as exc:
                self.get_logger().warn(
                    f"Gripper connection failed: {exc} — retrying in {RECONNECT_DELAY}s"
                )
                time.sleep(RECONNECT_DELAY)

    def _send_raw(self, cmd: str) -> str:
        with self._sock_lock:
            if self._sock is None:
                return ""
            try:
                self._sock.sendall((cmd + "\n").encode())
                time.sleep(0.02)
                self._sock.settimeout(1.0)
                return self._sock.recv(1024).decode().strip()
            except Exception:
                self._connected = False
                self._sock = None
        self._connect()
        return ""

    # ----------------------------------------------------------------------
    # Movement — FAST, NO WAITING
    # ----------------------------------------------------------------------
    def _move_to_pos(self, target_pos: int):
        # Send command immediately, no waiting, no polling
        self._send_raw(f"SET POS {target_pos}")
        self._current_pos = target_pos

    # ----------------------------------------------------------------------
    # State publishing
    # ----------------------------------------------------------------------
    def _publish_state(self):
        # Convert POS → joint value (binary)
        rad = JOINT_OPEN if self._current_pos == POS_OPEN else JOINT_CLOSED

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name         = [JOINT_NAME]
        js.position     = [rad]
        self._js_pub.publish(js)

        b = Bool()
        b.data = (self._current_pos == POS_CLOSED)
        self._bool_pub.publish(b)

        s = String()
        s.data = "open" if self._current_pos == POS_OPEN else "close"
        self._named_pub.publish(s)

    # ----------------------------------------------------------------------
    # Action server
    # ----------------------------------------------------------------------
    def _goal_callback(self, goal_request):
        if JOINT_NAME not in goal_request.trajectory.joint_names:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _):
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle: ServerGoalHandle):
        traj   = goal_handle.request.trajectory
        joints = list(traj.joint_names)
        idx    = joints.index(JOINT_NAME)

        # Only use the LAST point (fastest)
        cmd = traj.points[-1].positions[idx]

        # Binary open/close
        if cmd > (JOINT_OPEN + JOINT_CLOSED) / 2:
            self._move_to_pos(POS_OPEN)
        else:
            self._move_to_pos(POS_CLOSED)

        goal_handle.succeed()
        return FollowJointTrajectory.Result()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = RobotiqHandEBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()