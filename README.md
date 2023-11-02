# Thermal Human Pose Tracking ROS
This ROS package provides functionality for tracking human poses in Long-Wave Infrared (LWIR) images using MediaPipe library. The package consists of a launch script and a Python node for processing LWIR images and publishing human pose information. It has been tested on the [Seek Thermal Starter Kit](https://www.thermal.com/oem.html) using `Tyrian` and `Black Hot` color palettes.

![rviz_seek_mediapipe](https://github.com/wessamhamid/thermal_pose_tracking_ros/blob/main/docs/rviz_seek_mediapipe.png)

## Installation
To use this package, clone the repository into your ROS workspace and build the package.

```txt
mkdir catkin_ws
cd catkin_ws/src
git clone https://github.com/wessamhamid/thermal_pose_tracking_ros.git
```

## Dependencies
This package has the following dependencies:
- ROS (Robot Operating System)
- Python 3.X
- [opencv-python](https://github.com/opencv/opencv-python):
```txt
pip3 install opencv-python==4.2.0.32
```
- [Mediapipe](https://developers.google.com/mediapipe):
```txt
pip3 install mediapipe
```
- [seek_thermal_ros](https://github.com/wessamhamid/seek_thermal_ros):

## Usage
### Launching the LWIR Pose Tracking Node
To launch the LWIR pose tracking node, use the provided launch file:
```txt
roslaunch thermal_pose_tracking_ros lwir_pose_tracking.launch
```

## Details
The launch file (lwir_pose_tracking.launch) starts the `lwir_human_tracking_node` which subscribes to the `/seek_thermal_image` topic, processes LWIR images and tracks human poses using the MediaPipe library, and publishes the annotated images with human pose information on the `/lwir/human_tracking` topic.

### Subscribed Topics
/seek_thermal_image (sensor_msgs/Image): LWIR images from the Seek Thermal camera.
### Published Topics
/lwir/human_tracking (sensor_msgs/Image): Annotated LWIR images with human pose information.
### Parameters
- min_detection_confidence (default: 0.5): Minimum confidence threshold for landmark detection.
- min_tracking_confidence (default: 0.5): Minimum confidence threshold for landmark tracking.
