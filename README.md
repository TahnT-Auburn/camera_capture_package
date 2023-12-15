# camera_capture_package

## Description:
ROS2 package to stream and capture images on a LUCID Vision Camera.

This package uses the Arena SDK and OpenCV libraries to enumerate device, set nodes, stream and save images.

**For more information on this package, check the Joplin documentation under the _Camera Capture Package_ page.**

## Requirments:
* Ubuntu Focal Fossa
* ROS2 Foxy Fitzroy
* Arena SDK for Linux
* C++17 or higher

## To Use:
**_Before Use:_** 
* **Make sure ALL PATHS ARE SET CORRECTLY in the launch and config files before use!**
* **These steps assume you have already created a workspace folder and a `/src` directory within it!**

**_Steps:_**
1. Navigate into the `/src` directory of your workspace and clone the repo using `git clone`
2. Navigate back into the workspace directory and source `$ source /opt/ros/foxy/setup.bash`
3. Build package `$ colcon build` or `$ colcon build --packages-select <package_name>`
4. Open a new terminal and source it `$ . install/setup.bash`
5. Run launch file `$ ros2 launch <package_name> <launch_file_name>` in this case it is `$ ros2 launch camera_capture_package camera_capture_launch.py`