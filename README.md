# Setup
- Ubuntu 22.04
- ROS 2 Humble

# Requirements & Install
In your workspace, you should have the [realsense-ros package](https://github.com/IntelRealSense/realsense-ros/) in your `src` folder. Clone this repository into the same `src` folder and run `colcon build` in your workspace directory.

# Usage
Edit [multi_camera_launch.py](./realsense_camera_stats/launch/multi_camera_launch.py) where required.

`ros2 launch realsense_camera_stats multi_camera_launch.py` should work... I'll make a more detailed README when I'm not as busy D:


# Configurations
Running ROS2 params to find out all the configurations you can set in the launch file:

`ros2 param describe <node> depth_module.profile`
`ros2 param describe <node> rgb_camera.profile`

and we will find these configurations:

| Resolution | Frame Rate (fps) | Available In        |
|------------|------------------|---------------------|
| 1280x720   | 5, 10*, 15, 30   | Both (*RGB Camera only) |
| 1280x800   | 5, 10*, 15, 25*, 30 | Both (*RGB Camera only) |
| 256x144    | 90               | Depth Module only   |
| 424x240    | 5, 15, 30, 60, 90| Both                |
| 480x270    | 5, 15, 30, 60, 90| Both                |
| 640x360    | 5, 15, 30, 60, 90| Both                |
| 640x400    | 15, 25           | Depth Module only   |
| 640x480    | 5, 15, 30, 60, 90| Both                |
| 848x100    | 100              | Depth Module only   |
| 848x480    | 5, 15, 30, 60, 90| Both                |


From this, you can mix and match the configurations and test out which are stable or not. Perhaps I should've written an algorithm to determine "stability" but for now I'm relying on 30 second tests to see how bad the frame drops are.