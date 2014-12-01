/*
 * kinect_tilt_calibration_node.cpp
 *
 *  Created on: Nov 27, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>
#include "kinect_tilt_calibration/KinectTiltCalibration.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinect_tilt_calibration");

  ros::NodeHandle nodeHandle("~");

  kinect_tilt_calibration::KinectTiltCalibration kinectTiltCalibration(nodeHandle);

  ros::spin();
  return 0;
}
