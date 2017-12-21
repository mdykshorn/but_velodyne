but_calibration_camera_velodyne
===============================

ROS package for transforming a velodyne point cloud and an RGB image to a XYZRGB point cloud

Requirements
------------

* ROS Kinetic

Usage
-----

- `roslaunch but_calibration_camera_velodyne coloring.launch`
    - launches the publisher of colored pointclouds created by the camera -- Velodyne fusion

Configuration:
--------------

Configuration *.yaml files are stored in the directory `conf/`:

- `conf/calibration.yaml` contains folowing parameters:
    - `camera_frame_topic`: name of the topic, where camera RGB image is published
    - `camera_info_topic`: topic of `CameraInfo` type
    - `velodyne_topic`: topic, where the Velodyne pointcloud is published
    - `velodyne_color_topic`: name of the topic, where the colored pointcloud will be published
    - 'pcl_frame_id' : name of frame id to transform the pointcloud to
