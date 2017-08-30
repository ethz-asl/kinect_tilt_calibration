/*
 * KinectTiltCalibration.cpp
 *
 *  Created on: Nov 27, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "kinect_tilt_calibration/KinectTiltCalibration.hpp"

// ROS
#include <geometry_msgs/TransformStamped.h>

// STD
#include <string>

// PCL
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// Kindr
#include <kindr/Core>
#include <kindr_ros/kindr_ros.hpp>

using namespace std;

namespace kinect_tilt_calibration {

KinectTiltCalibration::KinectTiltCalibration(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      pointCloud_(new PointCloud),
      nSnapshots_(0)
{
  ROS_INFO("Kinect tilt calibration node started.");
  readParameters();
  pointCloudSubscriber_ = nodeHandle_.subscribe(pointCloudTopic_, 1, &KinectTiltCalibration::addPointCloud, this);
  pointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("point_cloud_inliers", 1, true);
}

KinectTiltCalibration::~KinectTiltCalibration() {}

bool KinectTiltCalibration::readParameters()
{
  nodeHandle_.param("topic", pointCloudTopic_, string("/camera/depth/points"));
  nodeHandle_.param("distance_threshold", distanceThreshold_, 0.01);
  nodeHandle_.param("tilting_frame", tiltingFrameId_, string("camera_link"));

  int maxSnapshotsTemp;
  nodeHandle_.param("number_of_snapshots", maxSnapshotsTemp, 10);
  if (maxSnapshotsTemp < 0) {
    ROS_ERROR("'number_of_point_clouds' is smaller then 0!");
    return false;
  }
  maxSnapshots_ = (unsigned int) maxSnapshotsTemp;

  return true;
}

void KinectTiltCalibration::addPointCloud(const PointCloud& pointCloud)
{
  if (nSnapshots_ >= maxSnapshots_) return;
  ++nSnapshots_;
  *pointCloud_ += pointCloud;
  pointCloud_->header = pointCloud.header;
  ROS_INFO_STREAM("Added " << nSnapshots_ << " point clouds.");

  if (nSnapshots_ == maxSnapshots_) {
    calibrate();
    exit();
  }
}

bool KinectTiltCalibration::calibrate()
{
  ROS_INFO_STREAM("Collected " << pointCloud_->width << " points.");

  // Find biggest plane.
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> segmentation;
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(distanceThreshold_);
  segmentation.setInputCloud(pointCloud_);
  segmentation.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0) {
    ROS_ERROR("Could not estimate a planar model for the dataset.");
    return false;
  }

  ROS_INFO_STREAM("Calibrating with " << inliers->indices.size() << " inlier points.");
  Eigen::Vector3d normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
  ROS_DEBUG_STREAM("Surface normal in sensor frame (" << pointCloud_->header.frame_id << "): " << normal.transpose());
  if (!computeRotation(normal, pointCloud_->header.frame_id)) return false;

  ROS_INFO("Calibration done.");
  publishInliers(inliers);
  return true;
}

bool KinectTiltCalibration::computeRotation(const Eigen::Vector3d& surfaceNormal, const std::string& frameId)
{
  // Get surface normal in tilting frame.
  tf::StampedTransform transform;
  try {
    tfListener_.lookupTransform(tiltingFrameId_, frameId, ros::Time(0), transform);
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }
  kindr::HomTransformQuatD pose;
  kindr_ros::convertFromRosTf(transform, pose);
  ROS_DEBUG_STREAM("Pose of sensor frame in tilting frame: " << endl << pose << endl);
  Eigen::Vector3d surfaceNormalInTiltingFrame = pose.getRotation().rotate(surfaceNormal);
    if (surfaceNormalInTiltingFrame.z() < 0.0) surfaceNormalInTiltingFrame = -surfaceNormalInTiltingFrame;
  ROS_DEBUG_STREAM("Surface normal in tilting frame (" << tiltingFrameId_ << "): " << surfaceNormalInTiltingFrame.transpose());

  // Compute calibration angles.
  Eigen::Vector3d reference = Eigen::Vector3d::UnitZ();
  kindr::RotationQuaternionPD rotation;
  rotation.setFromVectors(surfaceNormalInTiltingFrame, reference);

  cout << "===============================" << endl;
  cout << "Quaternion (qx, qy, qz, qw): " << rotation.x() << ", " << rotation.y() << ", " << rotation.z() << ", " << rotation.w() << endl;
  kindr::EulerAnglesYprPD euler(rotation);
  Eigen::Vector3d eulerVector = euler.getUnique().toImplementation() / M_PI * 180.0;
  cout << "Pitch: " << eulerVector(1) << " deg, Roll: " << eulerVector(2) << " deg" << endl; // Yaw should be 0 up to numerical errors.
  cout << "===============================" << endl;

  return true;
}

void KinectTiltCalibration::publishInliers(pcl::PointIndices::ConstPtr inlierIndices)
{
  if (pointCloudPublisher_.getNumSubscribers() < 1) return;
  PointCloud inlierPointCloud;
  pcl::ExtractIndices<pcl::PointXYZ> extractIndicesFilter(true);
  extractIndicesFilter.setInputCloud(pointCloud_);
  extractIndicesFilter.setIndices(inlierIndices);
  extractIndicesFilter.filter(inlierPointCloud);
  ROS_INFO("Publishing inlier point cloud.");
  pointCloudPublisher_.publish(inlierPointCloud);
}

void KinectTiltCalibration::exit()
{
  if (pointCloudPublisher_.getNumSubscribers() > 0) {
    ros::Duration duration(1.0);
    duration.sleep();
  }
  ros::requestShutdown();
}

} /* namespace kinect_tilt_calibration */
