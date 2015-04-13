
#ifndef ODOMETER_BASE_H_
#define ODOMETER_BASE_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>

#include <tf/transform_broadcaster.h>

namespace viso2_ros {

/**
 * Base class for odometers, handles tf's, odometry and pose
 * publishing. This can be used as base for any incremental pose estimating
 * sensor. Sensors that measure velocities cannot be used.
 */
class OdometerBase {
 private:
  // publisher
  ros::Publisher odom_pub_, pose_pub_, pose_cov_pub_;

  ros::ServiceServer reset_service_;

  // tf related
  std::string sensor_frame_id_;
  std::string odom_frame_id_;
  tf::TransformBroadcaster tf_broadcaster_;
  bool publish_tf_;

  // the current integrated camera pose
  tf::Transform integrated_pose_;
  // timestamp of the last update
  ros::Time last_update_time_;

  // covariances
  boost::array<double, 36> pose_covariance_;
  boost::array<double, 36> twist_covariance_;

 public:
  OdometerBase() {
    // Read local parameters
    ros::NodeHandle pnh("~");

    pnh.param("odom_frame_id", odom_frame_id_, std::string("odom"));
    pnh.param("pub_tf", publish_tf_, true);

    ROS_INFO_STREAM("Basic Odometer Settings:"
                    << std::endl << "  odom_frame_id\t= " << odom_frame_id_
                    << std::endl
                    << "  publish_tf\t= " << (publish_tf_ ? "true" : "false"));

    // advertise
    odom_pub_ = pnh.advertise<nav_msgs::Odometry>("odometry", 1);
    pose_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("pose", 1);
    pose_cov_pub_ =
        pnh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_cov", 1);

    reset_service_ =
        pnh.advertiseService("reset_pose", &OdometerBase::resetPose, this);

    integrated_pose_.setIdentity();

    pose_covariance_.assign(0.0);
    twist_covariance_.assign(0.0);
  }

 protected:
  void setSensorFrameId(const std::string& frame_id) {
    sensor_frame_id_ = frame_id;
  }

  std::string getSensorFrameId() const { return sensor_frame_id_; }

  void setPoseCovariance(const boost::array<double, 36>& pose_covariance) {
    pose_covariance_ = pose_covariance;
  }

  void setTwistCovariance(const boost::array<double, 36>& twist_covariance) {
    twist_covariance_ = twist_covariance;
  }

  void integrateAndPublish(const tf::Transform& delta_transform,
                           const ros::Time& timestamp) {
    if (sensor_frame_id_.empty()) {
      ROS_ERROR("[odometer] update called with unknown sensor frame id!");
      return;
    }
    if (timestamp < last_update_time_) {
      ROS_WARN(
          "[odometer] saw negative time change in incoming sensor data, "
          "resetting pose.");
      integrated_pose_.setIdentity();
    }
    integrated_pose_ *= delta_transform;

    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.stamp = timestamp;
    odometry_msg.header.frame_id = odom_frame_id_;
    odometry_msg.child_frame_id = sensor_frame_id_;
    tf::poseTFToMsg(integrated_pose_, odometry_msg.pose.pose);

    odometry_msg.pose.covariance = pose_covariance_;
    odometry_msg.twist.covariance = twist_covariance_;
    odom_pub_.publish(odometry_msg);

    // Publish pose stamped
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = odometry_msg.header;
    pose_msg.pose = odometry_msg.pose.pose;
    pose_pub_.publish(pose_msg);

    // Publish pose with covariance stamped
    geometry_msgs::PoseWithCovarianceStamped pose_cov_msg;
    pose_cov_msg.header = odometry_msg.header;
    pose_cov_msg.pose = odometry_msg.pose;
    pose_cov_pub_.publish(pose_cov_msg);

    if (publish_tf_) {
      tf_broadcaster_.sendTransform(tf::StampedTransform(
          integrated_pose_, timestamp, odom_frame_id_, sensor_frame_id_));
    }

    last_update_time_ = timestamp;
  }

  bool resetPose(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    integrated_pose_.setIdentity();
    return true;
  }
};

}  // viso2_ros

#endif
