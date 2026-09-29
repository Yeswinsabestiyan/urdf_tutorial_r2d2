# urdf_tutorial_r2d2

A ROS 2 package that builds the classic R2D2 URDF robot (body, swiveling head, rolling legs/wheels, and moving gripper arm) and animates it in RViz2.

## Project Structure

```text
urdf_tutorial_r2d2/
└── urdf_tutorial_r2d2/
    ├── launch/
    │   └── display.launch.py
    ├── rviz/
    │   └── r2d2.rviz
    ├── urdf/
    │   └── r2d2.urdf.xacro
    ├── urdf_tutorial_r2d2/
    │   ├── __init__.py
    │   └── state_publisher.py
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    └── README.md
```

## Prerequisites

```bash
sudo apt install ros-<distro>-robot-state-publisher \
                 ros-<distro>-joint-state-publisher \
                 ros-<distro>-joint-state-publisher-gui \
                 ros-<distro>-xacro ros-<distro>-rviz2
```

## Build & Install

1. Copy or clone the `urdf_tutorial_r2d2` package folder into your ROS 2 workspace `src/` directory (e.g. `~/ros2_ws/src/`):
   ```bash
   cd ~/ros2_ws/src
   # Ensure the package folder is in src/
   ```

2. Build and source:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select urdf_tutorial_r2d2
   source install/setup.bash
   ```

## Run

To launch the robot with automated state publisher animation:

```bash
ros2 launch urdf_tutorial_r2d2 display.launch.py
```

### Manual Joint Control (Sliders)

To drag joint sliders manually instead of automated animation, edit `launch/display.launch.py` to use `joint_state_publisher_gui`:

```python
Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui',
),
```
