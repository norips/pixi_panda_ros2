# pixi_panda_ros2

ROS2 Jazzy environment for Franka Panda robot using [pixi](https://prefix.dev/docs/pixi/overview) along with lerobot environment to control Franka Panda using SO-ARM100.

Features:
- **Single robot focus** - streamlined for real robot operation
- **CRISP Controllers** - cartesian/joint impedance controllers from [crisp_controllers](https://github.com/utiasDSL/crisp_controllers)
- **pixi package management** - reproducible environment with robostack
- **lerobot teleoperate** - teleoperate Franka Panda using SO-ARM100

## Prerequisites

Install pixi:
```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

## Quick Start

This project is based on 3 components:
- Franka controller using ROS
- SO-ARM100 using lerobot
- Bridge lerobot ROS

First clone the repo:

```bash
# Clone with submodules
git clone --recurse-submodules https://github.com/lvjonok/pixi_panda_ros2.git
cd pixi_panda_ros2
```

Then enable Frnaka Panda FCI through the web interface.

### Franka controller using ROS

Enable Franka Panda control through ROS using CRISP.

```bash
# Setup: install dependencies, clone CRISP, and build
pixi run -e jazzy setup

# Launch the robot
pixi run -e jazzy franka robot_ip:=192.168.8.2
```

### SO-ARM100 using lerobot

Read SO-ARM100 end effector position and send it to the bridge

```bash
# Launch the SO-ARM100
pixi run -e lerobot teleoperate --port /dev/ttyACM0
```

###  Bridge lerobot ROS

Read SO-ARM100 end effector position and send it to the bridge

```bash
# Launch the SO-ARM100
pixi run -e jazzy send
```

## Available Commands

| Command | Description |
|---------|-------------|
| `pixi run -e jazzy setup` | Full setup: submodules, clone CRISP, build |
| `pixi run -e jazzy build` | Build packages with colcon |
| `pixi run -e jazzy franka robot_ip:=<IP>` | Launch the Franka robot |
| `pixi run -e jazzy rviz robot_ip:=<IP>` | Launch the Franka robot with rviz GUI |

## Launch Arguments

The `franka` command accepts:

| Argument | Default | Description |
|----------|---------|-------------|
| `robot_ip` | (required) | IP address of the robot |
| `load_gripper` | `true` | Load the Franka gripper |
| `use_fake_hardware` | `false` | Use fake hardware for testing |
| `use_rviz` | `false` | Launch RViz for visualization |
| `description_package` | `franka_description` | Package containing robot description |
| `description_file` | `robots/real/panda_arm.urdf.xacro` | Path to xacro file relative to package |

## CRISP Controllers

The following controllers are available (spawned inactive by default):
- `cartesian_impedance_controller`
- `joint_impedance_controller`
- `gravity_compensation`
- `pose_broadcaster`

Activate a controller:
```bash
ros2 control switch_controllers --activate cartesian_impedance_controller
```

## Packages

| Package | Description |
|---------|-------------|
| `libfranka` | C++ library for Franka robot (v0.9.2) |
| `franka_msgs` | ROS2 message definitions |
| `franka_description` | URDF/xacro models |
| `franka_hardware` | ros2_control hardware interface |
| `franka_control2` | Control node |
| `franka_gripper` | Gripper control |
| `franka_semantic_components` | Semantic component interfaces |
| `franka_robot_state_broadcaster` | State broadcasting |
| `franka_bringup` | Launch files and configs |
| `crisp_controllers` | CRISP impedance controllers (cloned) |

## Development

Enter the pixi environment:
```bash
pixi shell -e jazzy
```