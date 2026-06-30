import time
import json
import threading
import numpy as np
import zmq

from scipy.spatial.transform import Rotation

try:
    from crisp_py.robot import make_robot
except Exception as exc:
    make_robot = None
    _ROBOT_IMPORT_ERROR = exc
else:
    _ROBOT_IMPORT_ERROR = None

try:
    from crisp_py.gripper import make_gripper
except Exception as exc:
    make_gripper = None
    _GRIPPER_IMPORT_ERROR = exc
else:
    _GRIPPER_IMPORT_ERROR = None

from rclpy.action import ActionClient
from franka_msgs.action import Move


# ---------------------------------------------------------------------------
# Safety workspace bounding box
WORKSPACE_MIN = np.array([0.35, -0.40, 0.25], dtype=float)
WORKSPACE_MAX = np.array([0.70,  0.40, 0.70], dtype=float)
# ---------------------------------------------------------------------------


def clip_to_bounds(position, lo=WORKSPACE_MIN, hi=WORKSPACE_MAX, eps=1e-9):
    clipped = np.clip(position, lo, hi)
    delta = np.abs(clipped - position)
    clipped_axes = [ax for ax, d in zip("xyz", delta) if d > eps]
    was_clipped = len(clipped_axes) > 0
    return clipped, was_clipped, clipped_axes

class FrankaMoveGripper:
    """
    Async client for /panda_gripper/move.

    Input:
        width in meters, total distance between both fingers.
        0.00 = closed
        0.04 = 4 cm opening
        0.08 = fully open-ish
    """

    def __init__(self, node, action_name="/panda_gripper/move"):
        self.node = node
        self.client = ActionClient(node, Move, action_name)

        self.last_width = None
        self.last_send_time = 0.0

    def move_async(self, width, speed=1.0, min_delta=0.002, min_period=0.20):
        width = float(np.clip(width, 0.0, 0.08))
        now = time.monotonic()

        # Avoid spamming the gripper action server at 50 Hz
        if self.last_width is not None:
            if abs(width - self.last_width) < min_delta:
                return

        if now - self.last_send_time < min_period:
            return

        # if not self.client.wait_for_server(timeout_sec=0.05):
        #     print("[gripper] /panda_gripper/move action server not ready")
        #     return

        goal = Move.Goal()
        goal.width = width
        goal.speed = float(speed)

        print(f"[gripper] sending width={goal.width:.3f}, speed={goal.speed:.3f}")

        self.client.send_goal_async(goal)

        self.last_width = width
        self.last_send_time = now

class MoveToWorker(threading.Thread):
    """
    Dedicated thread for robot.move_to().

    The main loop can keep receiving ZMQ commands while robot.move_to()
    is blocking. If many commands arrive while the robot is moving,
    only the latest target is kept.
    """

    def __init__(self, robot, gripper, speed=0.2):
        super().__init__(daemon=True)
        self.robot = robot
        self.gripper = gripper
        self.speed = speed

        # Correct: use robot.node, not self
        self.franka_gripper = FrankaMoveGripper(robot.node)

        self._condition = threading.Condition()
        self._latest_position = None
        self._latest_gripper = None
        self._stop_requested = False

        ctrl_freq = 50.0
        self.rate = robot.node.create_rate(ctrl_freq)
        self.first_time = True

    def submit(self, position, gripper):
        position = np.array(position, dtype=float).copy()

        with self._condition:
            self._latest_position = position
            self._latest_gripper = gripper
            self._condition.notify()

    def stop(self):
        with self._condition:
            self._stop_requested = True
            self._condition.notify()

    def run(self):
        target_pose = self.robot.end_effector_pose.copy()
        target_pose.orientation = Rotation.from_matrix([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
        ])

        if self.gripper is not None and hasattr(self.gripper, "enable_torque"):
            try:
                self.gripper.enable_torque()
            except Exception as e:
                print(f"[gripper] enable_torque skipped/failed: {e}")

        while True:
            with self._condition:
                while self._latest_position is None and not self._stop_requested:
                    self._condition.wait()

                if self._stop_requested:
                    return

                position = self._latest_position
                self._latest_position = None

                gripper = self._latest_gripper
                self._latest_gripper = None

            try:
                if self.first_time:
                    self.robot.move_to(position=position, speed=0.2)
                    self.first_time = False
                else:
                    target_pose.position = np.array([
                        position[0],
                        position[1],
                        position[2],
                    ])

                    # Your ZMQ value is normalized 0.0 -> 1.0
                    # Franka action expects width in meters: 0.0 -> 0.08
                    gripper_width = float(np.clip(gripper, 0.0, 1.0)) * 0.08

                    self.franka_gripper.move_async(
                        width=gripper_width,
                        speed=1.0,
                    )

                    print(
                        f"Gripper normalized={gripper:.3f}, "
                        f"width={gripper_width:.3f} m"
                    )

                    self.robot.set_target(pose=target_pose)

                self.rate.sleep()

            except Exception as e:
                print(f"[send_to_ros] move_to/set_target failed: {e}")


def main():
    if np.any(WORKSPACE_MIN >= WORKSPACE_MAX):
        raise ValueError(
            f"Invalid WORKSPACE bounds: MIN {WORKSPACE_MIN} must be < MAX {WORKSPACE_MAX} on every axis"
        )

    if make_robot is None:
        raise RuntimeError(f"Could not import crisp_py.robot.make_robot: {_ROBOT_IMPORT_ERROR}")

    robot = make_robot("panda")

    if hasattr(robot, "wait_until_ready"):
        robot.wait_until_ready()
    robot.home()
    robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")

    gripper = None
    if make_gripper is not None:
        gripper = make_gripper("gripper_franka", max_delta=1.0, use_gripper_command_action = True)
        # gripper.wait_until_ready()
        print("[send_to_ros] CRISP gripper ready")
    else:
        print(f"[send_to_ros] Could not import crisp_py.gripper: {_GRIPPER_IMPORT_ERROR}")

    print(
        f"[send_to_ros] Safety box active: "
        f"x[{WORKSPACE_MIN[0]:.3f}, {WORKSPACE_MAX[0]:.3f}] "
        f"y[{WORKSPACE_MIN[1]:.3f}, {WORKSPACE_MAX[1]:.3f}] "
        f"z[{WORKSPACE_MIN[2]:.3f}, {WORKSPACE_MAX[2]:.3f}]"
    )

    # Start move_to worker thread
    move_worker = MoveToWorker(robot, gripper, speed=0.2)
    move_worker.start()

    print("Starting ZMQ")
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.CONFLATE, 1)
    socket.connect("ipc:///tmp/test.sock")
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
    print("ZMQ connected")

    scale_ratio = 1.8
    prev_gripper = None
    gripper_close_threshold = 0.01

    print("[send_to_ros] Starting command loop...")

    ctrl_freq = 50.0
    rate = robot.node.create_rate(ctrl_freq)
    try:
        while True:
            try:
                msg = socket.recv_string(flags=zmq.NOBLOCK)
                data = json.loads(msg)

                position = np.array([
                    data["ee.x"] * scale_ratio,
                    data["ee.y"] * scale_ratio,
                    data["ee.z"] * scale_ratio
                ], dtype=float)

                position, was_clipped, clipped_axes = clip_to_bounds(position)

                if was_clipped:
                    print(
                        f"[send_to_ros] WARNING: command clipped on {','.join(clipped_axes)} "
                        f"-> {np.round(position, 4).tolist()}"
                    )

                # Send target to move thread instead of blocking here
                move_worker.submit(position=position, gripper=data.get("ee.gripper_pos", 0) / 100.0)


                debug = {
                    "x": data["ee.x"],
                    "y": data["ee.y"],
                    "z": data["ee.z"],
                    "qx": data["ee.qx"],
                    "qy": data["ee.qy"],
                    "qz": data["ee.qz"],
                    "qw": data["ee.qw"],
                    "gripper": data.get("ee.gripper_pos", 0) / 100.0,
                    "target_robot": np.round(position, 4).tolist(),
                }

                print(f"[send_to_ros] Received command: {debug}")

            except zmq.Again:
                pass
            except Exception as e:
                print(f"[send_to_ros] Error: {e}")

            # Avoid 100% CPU busy loop
            rate.sleep()

    except KeyboardInterrupt:
        print("\n[send_to_ros] Stopping...")

    finally:
        move_worker.stop()
        move_worker.join(timeout=1.0)

        socket.close()
        context.term()

        print("[send_to_ros] Stopped cleanly")


if __name__ == "__main__":
    main()