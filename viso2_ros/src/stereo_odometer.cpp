#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/stereo_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <viso_stereo.h>

#include <viso2_ros/VisoInfo.h>

#include "stereo_processor.h"
#include "odometer_base.h"
#include "odometry_params.h"

// to remove after debugging
#include <opencv2/highgui/highgui.hpp>

namespace viso2_ros {

using namespace sensor_msgs;

cv::Scalar disparityToCvScalar(double d);

// some arbitrary values (0.1m^2 linear cov. 10deg^2. angular cov.)
static const boost::array<double, 36> STANDARD_POSE_COVARIANCE = {
    {0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.17,
     0, 0, 0, 0, 0, 0, 0.17, 0, 0, 0, 0, 0, 0, 0.17}};
static const boost::array<double, 36> STANDARD_TWIST_COVARIANCE = {
    {0.05, 0, 0, 0, 0, 0, 0, 0.05, 0, 0, 0, 0, 0, 0, 0.05, 0, 0, 0, 0, 0, 0,
     0.09, 0, 0, 0, 0, 0, 0, 0.09, 0, 0, 0, 0, 0, 0, 0.09}};
static const boost::array<double, 36> BAD_COVARIANCE = {
    {9999, 0, 0, 0, 0, 0, 0, 9999, 0, 0, 0, 0, 0, 0, 9999, 0, 0, 0, 0, 0, 0,
     9999, 0, 0, 0, 0, 0, 0, 9999, 0, 0, 0, 0, 0, 0, 9999}};

class StereoOdometer : public StereoProcessor, public OdometerBase {
 private:
  boost::shared_ptr<VisualOdometryStereo> stereo_odometer_;
  VisualOdometryStereo::parameters stereo_odometer_params_;

  ros::NodeHandle pnh_;
  ros::Publisher point_cloud_pub_;
  image_transport::ImageTransport it_;
  image_transport::Publisher pub_matches_;
  ros::Publisher info_pub_;

  bool vis_matches_;
  bool got_lost_;

  // change reference frame method. 0, 1 or 2. 0 means allways change. 1 and 2
  // explained below
  int ref_frame_change_method_;
  bool change_reference_frame_;
  // method 1. Change the reference frame if last motion is small
  // method 2. Change the reference frame if the number of inliers is low
  double ref_frame_motion_threshold_;
  int ref_frame_inlier_threshold_;
  Matrix reference_motion_;
  cv::Mat display_;

 public:
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

  explicit StereoOdometer(const std::string& transport)
      : StereoProcessor(transport),
        OdometerBase(),
        pnh_("~"),
        it_(pnh_),
        got_lost_(false),
        change_reference_frame_(false) {
    // Read local parameters
    odometry_params::loadParams(pnh_, stereo_odometer_params_);

    pnh_.param("ref_frame_change_method", ref_frame_change_method_, 0);
    pnh_.param("ref_frame_motion_threshold", ref_frame_motion_threshold_, 5.0);
    pnh_.param("ref_frame_inlier_threshold", ref_frame_inlier_threshold_, 150);
    pnh_.param("vis_matches", vis_matches_, false);

    point_cloud_pub_ = pnh_.advertise<PointCloud>("cloud2", 1);
    info_pub_ = pnh_.advertise<VisoInfo>("info", 1);

    if (vis_matches_) {
      pub_matches_ = it_.advertise("image_output", 1);
    }

    reference_motion_ = Matrix::eye(4);
  }

 protected:
  void initOdometer(const CameraInfoConstPtr& l_info_msg,
                    const CameraInfoConstPtr& r_info_msg) {
    setSensorFrameId(l_info_msg->header.frame_id);
    // read calibration info from camera info message
    // to fill remaining parameters
    image_geometry::StereoCameraModel model;
    model.fromCameraInfo(l_info_msg, r_info_msg);
    stereo_odometer_params_.base = model.baseline();
    stereo_odometer_params_.calib.f = model.left().fx();
    stereo_odometer_params_.calib.cu = model.left().cx();
    stereo_odometer_params_.calib.cv = model.left().cy();
    stereo_odometer_.reset(new VisualOdometryStereo(stereo_odometer_params_));
    if (l_info_msg->header.frame_id != "")
      setSensorFrameId(l_info_msg->header.frame_id);
    ROS_INFO_STREAM(
        "Initialized libviso2 stereo odometry "
        "with the following parameters:\n"
        << stereo_odometer_params_
        << "  ref_frame_change_method = " << ref_frame_change_method_
        << "\n  ref_frame_motion_threshold = " << ref_frame_motion_threshold_
        << "\n  ref_frame_inlier_threshold = " << ref_frame_inlier_threshold_
        << "\n  visualize matches = " << std::boolalpha << vis_matches_);
  }

  void imageCallback(const ImageConstPtr& l_image_msg,
                     const ImageConstPtr& r_image_msg,
                     const CameraInfoConstPtr& l_cinfo_msg,
                     const CameraInfoConstPtr& r_cinfo_msg) {
    ros::WallTime start_time = ros::WallTime::now();
    bool first_run = false;
    // create odometer if not exists
    if (!stereo_odometer_) {
      first_run = true;
      initOdometer(l_cinfo_msg, r_cinfo_msg);
    }

    // convert images if necessary
    uint8_t* l_image_data, *r_image_data;
    int l_step, r_step;
    cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr;
    l_cv_ptr = cv_bridge::toCvShare(l_image_msg, image_encodings::MONO8);
    l_image_data = l_cv_ptr->image.data;
    l_step = l_cv_ptr->image.step[0];
    r_cv_ptr = cv_bridge::toCvShare(r_image_msg, image_encodings::MONO8);
    r_image_data = r_cv_ptr->image.data;
    r_step = r_cv_ptr->image.step[0];

    ROS_ASSERT(l_step == r_step);
    ROS_ASSERT(l_image_msg->width == r_image_msg->width);
    ROS_ASSERT(l_image_msg->height == r_image_msg->height);

    int32_t dims[] = {static_cast<int32_t>(l_image_msg->width),
                      static_cast<int32_t>(l_image_msg->height), l_step};
    // on first run or when odometer got lost, only feed the odometer with
    // images without retrieving data
    if (first_run || got_lost_) {
      stereo_odometer_->process(l_image_data, r_image_data, dims);
      got_lost_ = false;
      // on first run publish zero once
      if (first_run) {
        tf::Transform delta_transform;
        delta_transform.setIdentity();
        integrateAndPublish(delta_transform, l_image_msg->header.stamp);
      }
    } else {
      bool success = stereo_odometer_->process(l_image_data, r_image_data, dims,
                                               change_reference_frame_);
      if (success) {
        Matrix motion = Matrix::inv(stereo_odometer_->getMotion());
        ROS_DEBUG("Found %i matches with %i inliers.",
                  stereo_odometer_->getNumberOfMatches(),
                  stereo_odometer_->getNumberOfInliers());
        ROS_DEBUG_STREAM("libviso2 returned the following motion:\n" << motion);
        Matrix camera_motion;
        // if image was replaced due to small motion we have to subtract the
        // last motion to get the increment
        if (change_reference_frame_) {
          camera_motion = Matrix::inv(reference_motion_) * motion;
        } else {
          // image was not replaced, report full motion from odometer
          camera_motion = motion;
        }
        reference_motion_ = motion;  // store last motion as reference

        tf::Matrix3x3 rot_mat(camera_motion.val[0][0], camera_motion.val[0][1],
                              camera_motion.val[0][2], camera_motion.val[1][0],
                              camera_motion.val[1][1], camera_motion.val[1][2],
                              camera_motion.val[2][0], camera_motion.val[2][1],
                              camera_motion.val[2][2]);
        tf::Vector3 t(camera_motion.val[0][3], camera_motion.val[1][3],
                      camera_motion.val[2][3]);
        tf::Transform delta_transform(rot_mat, t);

        setPoseCovariance(STANDARD_POSE_COVARIANCE);
        setTwistCovariance(STANDARD_TWIST_COVARIANCE);

        integrateAndPublish(delta_transform, l_image_msg->header.stamp);

        if (point_cloud_pub_.getNumSubscribers() > 0) {
          computeAndPublishPointCloud(l_cinfo_msg, l_image_msg, r_cinfo_msg,
                                      stereo_odometer_->getMatches(),
                                      stereo_odometer_->getInlierIndices());
        }

        if (vis_matches_ && pub_matches_.getNumSubscribers()) {
          visualizeMatches(l_cv_ptr->image, stereo_odometer_->getMatches(),
                           stereo_odometer_->getInlierIndices(), start_time);
          cv_bridge::CvImage cv_img(l_cv_ptr->header, image_encodings::BGR8,
                                    display_);
          pub_matches_.publish(cv_img.toImageMsg());
        }
      } else {
        setPoseCovariance(BAD_COVARIANCE);
        setTwistCovariance(BAD_COVARIANCE);
        tf::Transform delta_transform;
        delta_transform.setIdentity();
        integrateAndPublish(delta_transform, l_image_msg->header.stamp);

        ROS_DEBUG("Call to VisualOdometryStereo::process() failed.");
        ROS_WARN_THROTTLE(1.0, "Visual Odometer got lost!");
        got_lost_ = true;
      }

      if (success) {
        // Proceed depending on the reference frame change method
        switch (ref_frame_change_method_) {
          case 1: {
            // calculate current feature flow
            double feature_flow =
                computeFeatureFlow(stereo_odometer_->getMatches());
            change_reference_frame_ =
                (feature_flow < ref_frame_motion_threshold_);
            ROS_DEBUG_STREAM(
                "Feature flow is "
                << feature_flow << ", marking last motion as "
                << (change_reference_frame_ ? "small." : "normal."));
            break;
          }
          case 2: {
            change_reference_frame_ = (stereo_odometer_->getNumberOfInliers() >
                                       ref_frame_inlier_threshold_);
            break;
          }
          default:
            change_reference_frame_ = false;
        }
      } else {
        change_reference_frame_ = false;
      }

      if (!change_reference_frame_)
        ROS_DEBUG_STREAM("Changing reference frame");

      // create and publish viso2 info msg
      VisoInfo info_msg;
      info_msg.header.stamp = l_image_msg->header.stamp;
      info_msg.got_lost = !success;
      info_msg.change_reference_frame = !change_reference_frame_;
      info_msg.num_matches = stereo_odometer_->getNumberOfMatches();
      info_msg.num_inliers = stereo_odometer_->getNumberOfInliers();
      ros::WallDuration time_elapsed = ros::WallTime::now() - start_time;
      info_msg.runtime = time_elapsed.toSec();
      info_pub_.publish(info_msg);
    }
  }

  double computeFeatureFlow(const std::vector<Matcher::p_match>& matches) {
    double total_flow = 0.0;
    for (size_t i = 0; i < matches.size(); ++i) {
      double x_diff = matches[i].u1c - matches[i].u1p;
      double y_diff = matches[i].v1c - matches[i].v1p;
      total_flow += sqrt(x_diff * x_diff + y_diff * y_diff);
    }
    return total_flow / matches.size();
  }

  void visualizeMatches(const cv::Mat& l_image,
                        const std::vector<Matcher::p_match>& matches,
                        const std::vector<int32_t>& inlier_indices,
                        const ros::WallTime& start_time) {
    const auto n_rows = l_image.rows;
    // Convert grayscale to color and blend
    cv::cvtColor(l_image, display_, CV_GRAY2BGR);

    for (const auto& i : inlier_indices) {
      const Matcher::p_match& match = matches[i];
      const cv::Point2f l_uv(match.u1p, match.v1p);
      const cv::Point2f r_uv(match.u1c, match.v1c);
      const auto color = disparityToCvScalar(match.u1c - match.u2c);
      cv::circle(display_, l_uv, 2, color, -1, CV_AA);
      cv::line(display_, l_uv, r_uv, color, 1, CV_AA);
    }
    const auto time_used =
        static_cast<int>((ros::WallTime::now() - start_time).toSec() * 1000);
    cv::putText(display_, std::to_string(time_used) + "ms", cv::Point2f(15, 40),
                cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(0, 255, 255), 2, CV_AA);
    cv::putText(display_, std::to_string(inlier_indices.size()),
                cv::Point2f(15, n_rows - 20), cv::FONT_HERSHEY_SIMPLEX, 1,
                CV_RGB(0, 255, 255), 2, CV_AA);
  }

  void computeAndPublishPointCloud(const CameraInfoConstPtr& l_info_msg,
                                   const ImageConstPtr& l_image_msg,
                                   const CameraInfoConstPtr& r_info_msg,
                                   const std::vector<Matcher::p_match>& matches,
                                   const std::vector<int32_t>& inlier_indices) {
    try {
      cv_bridge::CvImageConstPtr cv_ptr;
      cv_ptr = cv_bridge::toCvShare(l_image_msg, image_encodings::RGB8);
      // read calibration info from camera info message
      image_geometry::StereoCameraModel model;
      model.fromCameraInfo(*l_info_msg, *r_info_msg);
      PointCloud::Ptr point_cloud(new PointCloud());
      point_cloud->header.frame_id = getSensorFrameId();
      point_cloud->header.stamp =
          pcl_conversions::toPCL(l_info_msg->header).stamp;
      point_cloud->width = 1;
      point_cloud->height = inlier_indices.size();
      point_cloud->points.resize(inlier_indices.size());

      for (size_t i = 0; i < inlier_indices.size(); ++i) {
        const Matcher::p_match& match = matches[inlier_indices[i]];
        cv::Point2d left_uv;
        left_uv.x = match.u1c;
        left_uv.y = match.v1c;
        cv::Point3d point;
        double disparity = match.u1c - match.u2c;
        model.projectDisparityTo3d(left_uv, disparity, point);
        point_cloud->points[i].x = point.x;
        point_cloud->points[i].y = point.y;
        point_cloud->points[i].z = point.z;
        cv::Vec3b color = cv_ptr->image.at<cv::Vec3b>(left_uv.y, left_uv.x);
        point_cloud->points[i].r = color[0];
        point_cloud->points[i].g = color[1];
        point_cloud->points[i].b = color[2];
      }
      ROS_DEBUG("Publishing point cloud with %zu points.", point_cloud->size());
      point_cloud_pub_.publish(point_cloud);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
};

cv::Scalar disparityToCvScalar(double d) {
  const static double dmax = 64;
  d = std::max(0.0, std::min(d, dmax));
  return cv::Scalar(0, 255 * (dmax - d) / dmax, 255 * d / dmax);
}

}  // end of namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "stereo_odometer");

  std::string transport = argc > 1 ? argv[1] : "raw";
  viso2_ros::StereoOdometer odometer(transport);

  ros::spin();
}
