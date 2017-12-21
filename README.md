# but_velodyne
============

ROS packages for Velodyne 3D LIDARs provided by Robo@FIT group.

These packages depend on [but_velodyne_lib](https://github.com/robofit/but_velodyne_lib) library which must be compiled and installed on the machine first. See the description of library repository for more information related to the compilation process. Do not forget to run `make install` after the library is built.

# Fork Changes

## but_calibration_camera_velodyne

we are not using the `calibration nodes`, however we are using `coloring node`

## Running

RGB Point Cloud creation
`roslaunch but_calibration_camera_velodyne coloring.launch`

TODO specifying topics

### currently the package but_velodyne_odom and but_velodyne_proc are not being compiled because we do not need them
