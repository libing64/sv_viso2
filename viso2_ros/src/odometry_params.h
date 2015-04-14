#include <ros/ros.h>

#include <viso_stereo.h>
#include <viso_mono.h>

namespace viso2_ros {

namespace odometry_params {

/// loads matcher params
void loadParams(const ros::NodeHandle& viso2_nh, Matcher::parameters& params) {
  viso2_nh.getParam("nms_n", params.nms_n);
  viso2_nh.getParam("nms_tau", params.nms_tau);
  viso2_nh.getParam("match_binsize", params.match_binsize);
  viso2_nh.getParam("match_radius", params.match_radius);
  viso2_nh.getParam("match_disp_tolerance", params.match_disp_tolerance);
  viso2_nh.getParam("outlier_disp_tolerance", params.outlier_disp_tolerance);
  viso2_nh.getParam("outlier_flow_tolerance", params.outlier_flow_tolerance);
  viso2_nh.getParam("multi_stage", params.multi_stage);
  viso2_nh.getParam("half_resolution", params.half_resolution);
  viso2_nh.getParam("refinement", params.refinement);
}

/// loads bucketing params
void loadParams(const ros::NodeHandle& viso2_nh,
                VisualOdometry::bucketing& bucketing) {
  viso2_nh.getParam("max_features", bucketing.max_features);
  viso2_nh.getParam("bucket_width", bucketing.bucket_width);
  viso2_nh.getParam("bucket_height", bucketing.bucket_height);
}

/// loads common odometry params
void loadCommonParams(const ros::NodeHandle& viso2_nh,
                      VisualOdometry::parameters& params) {
  loadParams(viso2_nh, params.match);
  loadParams(viso2_nh, params.bucket);
}

/// loads common & stereo specific params
void loadParams(const ros::NodeHandle& viso2_nh,
                VisualOdometryStereo::parameters& params) {
  loadCommonParams(viso2_nh, params);
  viso2_nh.getParam("ransac_iters", params.ransac_iters);
  viso2_nh.getParam("inlier_threshold", params.inlier_threshold);
  viso2_nh.getParam("reweighting", params.reweighting);
}

/// loads common & mono specific params
void loadParams(const ros::NodeHandle& viso2_nh,
                VisualOdometryMono::parameters& params) {
  loadCommonParams(viso2_nh, params);
  if (!viso2_nh.getParam("camera_height", params.height)) {
    ROS_WARN(
        "Parameter 'camera_height' is required but not set."
        "Using default: %f",
        params.height);
  }
  if (!viso2_nh.getParam("camera_pitch", params.pitch)) {
    ROS_WARN(
        "Paramter 'camera_pitch' is required but not set."
        "Using default: %f",
        params.pitch);
  }
  viso2_nh.getParam("ransac_iters", params.ransac_iters);
  viso2_nh.getParam("inlier_threshold", params.inlier_threshold);
  viso2_nh.getParam("motion_threshold", params.motion_threshold);
}

}  // end of namespace

std::ostream& operator<<(std::ostream& out, const Matcher::parameters& params) {
  out << "  nms_n                  = " << params.nms_n;
  out << "\n  nms_tau                = " << params.nms_tau;
  out << "\n  match_binsize          = " << params.match_binsize;
  out << "\n  match_radius           = " << params.match_radius;
  out << "\n  match_disp_tolerance   = " << params.match_disp_tolerance;
  out << "\n  outlier_disp_tolerance = " << params.outlier_disp_tolerance;
  out << "\n  outlier_flow_tolerance = " << params.outlier_flow_tolerance;
  out << "\n  multi_stage            = " << params.multi_stage;
  out << "\n  half_resolution        = " << params.half_resolution;
  out << "\n  refinement             = " << params.refinement << std::endl;
  return out;
}

std::ostream& operator<<(std::ostream& out,
                         const VisualOdometry::calibration& calibration) {
  out << "  f  = " << calibration.f;
  out << "\n  cu = " << calibration.cu;
  out << "\n  cv = " << calibration.cv << std::endl;
  return out;
}

std::ostream& operator<<(std::ostream& out,
                         const VisualOdometry::bucketing& bucketing) {
  out << "  max_features  = " << bucketing.max_features;
  out << "\n  bucket_width  = " << bucketing.bucket_width;
  out << "\n  bucket_height = " << bucketing.bucket_height << std::endl;
  return out;
}

std::ostream& operator<<(std::ostream& out,
                         const VisualOdometry::parameters& params) {
  out << "Calibration parameters:" << std::endl << params.calib;
  out << "Matcher parameters:" << std::endl << params.match;
  out << "Bucketing parameters:" << std::endl << params.bucket;
  return out;
}

std::ostream& operator<<(std::ostream& out,
                         const VisualOdometryStereo::parameters& params) {
  out << static_cast<VisualOdometry::parameters>(params);
  out << "Stereo odometry parameters:" << std::endl;
  out << "  base             = " << params.base << std::endl;
  out << "  ransac_iters     = " << params.ransac_iters << std::endl;
  out << "  inlier_threshold = " << params.inlier_threshold << std::endl;
  out << "  reweighting      = " << params.reweighting << std::endl;
  return out;
}

std::ostream& operator<<(std::ostream& out,
                         const VisualOdometryMono::parameters& params) {
  out << static_cast<VisualOdometry::parameters>(params);
  out << "Mono odometry parameters:" << std::endl;
  out << "  camera_height    = " << params.height << std::endl;
  out << "  camera_pitch     = " << params.pitch << std::endl;
  out << "  ransac_iters     = " << params.ransac_iters << std::endl;
  out << "  inlier_threshold = " << params.inlier_threshold << std::endl;
  out << "  motion_threshold = " << params.motion_threshold << std::endl;
  return out;
}

}  // end of namespace
