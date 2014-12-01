Kinect Tilt Calibration
======================

Overview
---------------

This is a simple [ROS] node to calibrate the tilt angles (pitch and roll) of a Kinect-like depth sensor. It works by averaging a plane from the depth data and computing the rotation that aligns this plane with the world horizontal plane.  

The Kinect tilt calibration has been tested under ROS Indigo and Ubuntu 14.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

**Author: Peter Fankhauser, pfankhauser@ethz.ch<br />
Affiliation: Autonomous Systems Lab, ETH Zurich**


Installation
------------

### Dependencies

This software is built on the Robotic Operating System ([ROS]), which needs to be [installed](http://wiki.ros.org) first. Additionaly, the it depends on following software:

- [Point Cloud Library (PCL)](http://pointclouds.org/),
- [kindr](http://github.com/ethz-asl/kindr) (kinematics and dynamics library for robotics),
- [Eigen](http://eigen.tuxfamily.org) (linear algebra library).


### Building

In order to install the Kinect tilt calibration, clone the latest version from this repository into your catkin workspace and compile the package using ROS.

    cd catkin_workspace/src
    git clone https://github.com/ethz-asl/kinect_tilt_calibration.git
    cd ../
    catkin_make


Usage
------------

Place your robot on a flat floor in its nominal configuration and make sure that all the Kinect sensor captures is the floor. Copy the launch-file from `launch/example.launch` and adapt the parameters.

    roslaunch kinect_tilt_calibration your_launch_file.launch


Nodes
------------

### Node: kinect_tilt_calibration

#### Subscribed Topics

* **`/camera/depth/points`** ([sensor_msgs/PointCloud2])

    The point cloud messages from the distance sensor.


#### Published Topics

* **`point_cloud_inliers`** ([sensor_msgs/PointCloud2])

    The aggregated point cloud used for the plane estimation.


#### Parameters

* **`topic`** (string, default: "/camera/depth/points")
 
    The name of the point cloud messages from the distance sensor.

* **`tilting_frame`** (string, default: "camera_link")
 
	Reference frame id of the frame for which the rotation should be estimated. This does not have to be the frame in which the point clouds are published. *Note: The parent from of the `tilting_frame` is assumed to be aligned with the world vertical axis.*

* **`distance_threshold`** (double, default: 0.01)
 
	The distance threshold to determine plane segmentation [m].
    
* **`number_of_snapshots`** (int, default: 10)
 
    The number of point clouds to average for calibration


Bugs & Feature Requests
------------

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/kinect_tilt_calibration/issues).


[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[sensor_msgs/PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
