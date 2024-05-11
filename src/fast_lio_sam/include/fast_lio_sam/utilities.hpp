#ifndef FAST_LIO_SAM_UTILITY_H
#define FAST_LIO_SAM_UTILITY_H

///// common headers
#include <string>
///// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
///// PCL
#include <pcl/point_types.h> //pt
#include <pcl/point_cloud.h> //cloud
#include <pcl/conversions.h> //ros<->pcl
#include <pcl_conversions/pcl_conversions.h> //ros<->pcl
#include <pcl/common/transforms.h>
///// Eigen
#include <Eigen/Eigen> // whole Eigen library: Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
///// GTSAM
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

using namespace std;
//////////////////////////////////////////////////////////////////////
///// conversions
gtsam::Pose3 pose_eig_to_gtsam_pose(const Eigen::Matrix4d &pose_eig_in)
{
	double r_, p_, y_;
	tf2::Matrix3x3 mat_(pose_eig_in(0, 0), pose_eig_in(0, 1), pose_eig_in(0, 2),
						pose_eig_in(1, 0), pose_eig_in(1, 1), pose_eig_in(1, 2),
						pose_eig_in(2, 0), pose_eig_in(2, 1), pose_eig_in(2, 2));
	mat_.getRPY(r_, p_, y_);
	return gtsam::Pose3(gtsam::Rot3::RzRyRx(r_, p_, y_), gtsam::Point3(pose_eig_in(0, 3), pose_eig_in(1, 3), pose_eig_in(2, 3)));
}
Eigen::Matrix4d gtsam_pose_to_pose_eig(const gtsam::Pose3 &gtsam_pose_in)
{
	Eigen::Matrix4d pose_eig_out_ = Eigen::Matrix4d::Identity();
	tf2::Quaternion quat_;
	quat_.setRPY(gtsam_pose_in.rotation().roll(), gtsam_pose_in.rotation().pitch(), gtsam_pose_in.rotation().yaw());
	tf2::Matrix3x3 mat_(quat_);
	pose_eig_out_(0, 0) = mat_[0][0]; pose_eig_out_(0, 1) = mat_[0][1]; pose_eig_out_(0, 2) = mat_[0][2];
	pose_eig_out_(1, 0) = mat_[1][0]; pose_eig_out_(1, 1) = mat_[1][1]; pose_eig_out_(1, 2) = mat_[1][2];
	pose_eig_out_(2, 0) = mat_[2][0]; pose_eig_out_(2, 1) = mat_[2][1]; pose_eig_out_(2, 2) = mat_[2][2];
	pose_eig_out_(0, 3) = gtsam_pose_in.translation().x();
	pose_eig_out_(1, 3) = gtsam_pose_in.translation().y();
	pose_eig_out_(2, 3) = gtsam_pose_in.translation().z();
	return pose_eig_out_;
}
geometry_msgs::msg::PoseStamped pose_eig_to_pose_stamped(const Eigen::Matrix4d &pose_eig_in, string frame_id="map")
{
	double r_, p_, y_;
	tf2::Matrix3x3 mat_(pose_eig_in(0, 0), pose_eig_in(0, 1), pose_eig_in(0, 2),
						pose_eig_in(1, 0), pose_eig_in(1, 1), pose_eig_in(1, 2),
						pose_eig_in(2, 0), pose_eig_in(2, 1), pose_eig_in(2, 2));
	mat_.getRPY(r_, p_, y_);
	tf2::Quaternion quat_;
	quat_.setRPY(r_, p_, y_);
	geometry_msgs::msg::PoseStamped pose_;
	pose_.header.frame_id = frame_id;
	pose_.pose.position.x = pose_eig_in(0, 3);
	pose_.pose.position.y = pose_eig_in(1, 3);
	pose_.pose.position.z = pose_eig_in(2, 3);
	pose_.pose.orientation.w = quat_.getW();
	pose_.pose.orientation.x = quat_.getX();
	pose_.pose.orientation.y = quat_.getY();
	pose_.pose.orientation.z = quat_.getZ();
	return pose_;
}
geometry_msgs::msg::PoseStamped gtsam_pose_to_pose_stamped(const gtsam::Pose3 &gtsam_pose_in, string frame_id="map")
{
	tf2::Quaternion quat_;
	quat_.setRPY(gtsam_pose_in.rotation().roll(), gtsam_pose_in.rotation().pitch(), gtsam_pose_in.rotation().yaw());
	geometry_msgs::msg::PoseStamped pose_;
	pose_.header.frame_id = frame_id;
	pose_.pose.position.x = gtsam_pose_in.translation().x();
	pose_.pose.position.y = gtsam_pose_in.translation().y();
	pose_.pose.position.z = gtsam_pose_in.translation().z();
	pose_.pose.orientation.w = quat_.getW();
	pose_.pose.orientation.x = quat_.getX();
	pose_.pose.orientation.y = quat_.getY();
	pose_.pose.orientation.z = quat_.getZ();
	return pose_;
}
template <typename T>
sensor_msgs::msg::PointCloud2 pcl_to_pcl_ros(pcl::PointCloud<T> cloud, string frame_id="map")
{
  sensor_msgs::msg::PointCloud2 cloud_ROS;
  pcl::toROSMsg(cloud, cloud_ROS);
  cloud_ROS.header.frame_id = frame_id;
  return cloud_ROS;
}

///// transformation
template <typename T>
pcl::PointCloud<T> tf_pcd(const pcl::PointCloud<T> &cloud_in, const Eigen::Matrix4d &pose_tf)
{
	if (cloud_in.size() == 0) return cloud_in;
	pcl::PointCloud<T> pcl_out_ = cloud_in;
	pcl::transformPointCloud(cloud_in, pcl_out_, pose_tf);
	return pcl_out_;
}



#endif