# urdf_tutorial_r2d2

A ROS 2 package that builds the classic R2D2 URDF tutorial robot (body,
swiveling head, two rolling legs/wheels, and a moving gripper arm) and
animates it live in RViz2 — same idea as the screenshot, but ROS 2.

## Prerequisites

```bash
sudo apt install ros-<distro>-robot-state-publisher \
                  ros-<distro>-joint-state-publisher \
                  ros-<distro>-joint-state-publisher-gui \
                  ros-<distro>-xacro ros-<distro>-rviz2
```

## Install

1. Copy the `urdf_tutorial_r2d2` folder into the `src` directory of a ROS 2
   workspace, e.g. `~/ros2_ws/src/urdf_tutorial_r2d2`.
2. Build:

```bash
cd ~/ros2_ws
colcon build --packages-select urdf_tutorial_r2d2
source install/setup.bash
```

## Run (robot moves on its own)

```bash
ros2 launch urdf_tutorial_r2d2 display.launch.py
```

This starts `robot_state_publisher`, a custom `state_publisher` node that
animates the wheels/head/gripper joints and drives `base_link` in a circle
around `odom`, and RViz2 pre-configured to show it (Fixed Frame = `odom`,
Grid + RobotModel + TF displays, same as your screenshot).

## Alternative: move it yourself with sliders

If you'd rather drag joint sliders instead of the automatic animation,
swap the `state_publisher` node in `launch/display.launch.py` for:

```python
Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui',
),
```
