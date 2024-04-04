# Setup
- Ubuntu 22.04
- ROS 2 Humble

# Requirements & Install
In your workspace, you should have the [realsense-ros package](https://github.com/IntelRealSense/realsense-ros/) in your `src` folder. Clone this repository into the same `src` folder and run `colcon build` in your workspace directory.

# Usage
Edit [multi_camera_launch.py](./realsense_camera_stats/launch/multi_camera_launch.py) where required.

`ros2 launch realsense_camera_stats multi_camera_launch.py` should work... I'll make a more detailed README when I'm not as busy D:
