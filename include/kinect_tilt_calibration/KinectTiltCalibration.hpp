/*
 * KinectTiltCalibration.hpp
 *
 *  Created on: Nov 27, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

// PCL
#include <pcl/point_types.h>

// Eigen
#include <Eigen/Core>

namespace kinect_tilt_calibration {

class KinectTiltCalibration
{
 public:
  KinectTiltCalibration(ros::NodeHandle& nodeHandle);
  virtual ~KinectTiltCalibration();

  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  /*!
   * Add point cloud to be considered in the calibration.
   * @param pointCloud the point cloud to be added.
   */
  void addPointCloud(const PointCloud& pointCloud);

  /*!
   * Start calibration procedure.
   * @return true if successful, false otherwise.
   */
  bool calibrate();

  /*!
   * Computes the rotation to align the surface normal given in frameId with the vertical.
   * @param surfaceNormal the surface normal to align
   * @param frameId the frame id of the surface normal.
   * @return true if successful, false otherwise.
   */
  bool computeRotation(const Eigen::Vector3d& surfaceNormal, const std::string& frameId);

 private:

  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Publish the inlier point cloud for visualization.
   * @param inlierIndices the indices of the inliers in the point cloud.
   */
  void publishInliers(pcl::PointIndices::ConstPtr inlierIndices);

  /*!
   * Quits the node.
   */
  void exit();

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! Point cloud subscriber.
  ros::Subscriber pointCloudSubscriber_;

  //! Point cloud topic.
  std::string pointCloudTopic_;

  //! Point cloud publisher for inlier visualization.
  ros::Publisher pointCloudPublisher_;

  //! Tf listener.
  tf::TransformListener tfListener_;

  //! Number of point clouds to average for calibration.
  unsigned int maxSnapshots_;

  //! Current number of point clouds.
  unsigned int nSnapshots_;

  //! Aggregated point cloud.
  PointCloud::Ptr pointCloud_;

  //! Distance threshold to determine plane segmentation.
  double distanceThreshold_;

  //! Reference frame id of the frame for which the
  //! rotation should be estimated. This does not have to be
  //! the frame in which the point clouds are published.
  std::string tiltingFrameId_;
};

} /* namespace kinect_tilt_calibration */
