import time
import math
import json
import numpy as np
import zmq

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

# Retrieve position through ZMQ and send commands to robot using crisp_py
def main():
    # Initialize CRISP robot and gripper
    if make_robot is None:
        raise RuntimeError(f"Could not import crisp_py.robot.make_robot: {_ROBOT_IMPORT_ERROR}")
    
    robot = make_robot("panda")
    if hasattr(robot, "wait_until_ready"):
        robot.wait_until_ready()
    
    gripper = None
    if make_gripper is not None:
        gripper = make_gripper("gripper_franka")
        gripper.wait_until_ready()
        print("[send_to_ros] CRISP gripper ready")
    else:
        print(f"[send_to_ros] Could not import crisp_py.gripper: {_GRIPPER_IMPORT_ERROR}")

    # ZMQ setup to receive pose commands from teleoperate.py
    context = zmq.Context()
    socket = context.socket(zmq.PAIR)
    socket.setsockopt(zmq.LINGER, 0)
    socket.connect("ipc:///tmp/test.sock")
    
    # Used to match so-100 leader range to Franka Panda range
    scale_ratio = 1.8
    prev_gripper = None
    gripper_close_threshold = 0.01  # Width threshold for close/open

    print("[send_to_ros] Starting command loop...")
    while True:
        try:
            msg = socket.recv_string(flags=zmq.NOBLOCK)

            # dict_keys(['ee.x', 'ee.y', 'ee.z', 'ee.wx', 'ee.wy', 'ee.wz', 'ee.qx', 'ee.qy', 'ee.qz', 'ee.qw'])
            data = json.loads(msg)
            
            # Extract position and scale
            position = np.array([
                data["ee.x"] * scale_ratio,
                data["ee.y"] * scale_ratio,
                data["ee.z"] * scale_ratio
            ], dtype=float)
            
            # Send move command to robot
            try:
                robot.move_to(position=position, speed=0.2)
            except Exception as e:
                print(f"[send_to_ros] move_to failed: {e}")
            
            # Handle gripper if data is available
            if "ee.gripper_pos" in data and gripper is not None:
                gripper_pos = data["ee.gripper_pos"]
                
                if prev_gripper is None:
                    prev_gripper = gripper_pos
                
                # Check if gripper command changed significantly
                if abs(prev_gripper - gripper_pos) > 5:
                    # Normalize gripper_pos from (-100, 100) or (0, 100) range to (0, 1)
                    normalized_width = np.clip((gripper_pos + 100) / 200, 0.0, 1.0)
                    
                    if normalized_width > gripper_close_threshold:
                        try:
                            print(f"[send_to_ros] Opening gripper")
                            gripper.open()
                        except Exception as e:
                            print(f"[send_to_ros] gripper.open() failed: {e}")
                    else:
                        try:
                            print(f"[send_to_ros] Closing gripper")
                            gripper.close()
                        except Exception as e:
                            print(f"[send_to_ros] gripper.close() failed: {e}")
                    
                    prev_gripper = gripper_pos

            debug = {
                "qx": data["ee.qx"],
                "qy": data["ee.qy"],
                "qz": data["ee.qz"],
                "qw": data["ee.qw"],
                "gripper": data.get("ee.gripper_pos", 0) / 100.0
            }
            print(f"[send_to_ros] Received command: {debug}")
            
        except zmq.Again:
            pass
        except Exception as e:
            print(f"[send_to_ros] Error: {e}")
        
        time.sleep(0.01)  # Control loop frequency

if __name__ == "__main__":
    main()