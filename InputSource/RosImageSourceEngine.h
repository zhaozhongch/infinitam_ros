#pragma once

#include "ImageSourceEngine.h"

#include <mutex>
#include <string>
#include <thread>
#if (!defined USING_CMAKE) && (defined _MSC_VER)
#ifdef _DEBUG
#pragma comment(lib, "libpxcmd_d")
#else
#pragma comment(lib, "libpxcmd")
#endif
#endif

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <tf/transform_broadcaster.h>
#include "../ITMLib/ITMLibDefines.h"
#include <queue>

namespace InputSource {

class RosImageSourceEngine : public BaseImageSourceEngine {
 private:
  cv_bridge::CvImagePtr cv_rgb_image_;
  cv_bridge::CvImagePtr cv_depth_image_;

  //! ROS topic name for the incoming rgb messages.
  //std::string rgb_camera_info_topic_;
  //! ROS Topic name for the incoming depth messages.
  //std::string depth_camera_info_topic_;
  
  std::mutex rgb_mutex_;
  std::mutex depth_mutex_;
  Vector2i image_size_rgb_, image_size_depth_;

  //sensor_msgs::CameraInfo rgb_info_;
  //sensor_msgs::CameraInfo depth_info_;

  ros::Subscriber rgb_sub_;
  ros::Subscriber depth_sub_;
  //ros::Subscriber tf_sub_;
  std::string rgb_image_topic;
  std::string depth_image_topic;

  std::thread *spin_thread_;

  std::queue<cv_bridge::CvImagePtr> rgb_queue_;
  std::queue<cv_bridge::CvImagePtr> depth_queue_;

  double timestamp_sec_;

  double depth_scale_ = 1.0;

  /*!
   * Time stamp of the incoming images. This is used to synchronize the
   * incoming images with the pose estimation.
   */
  ros::Time depth_msg_time_stamp_;

 public:
  RosImageSourceEngine(ros::NodeHandle& nh, const char*& calibration_filename);

  ~RosImageSourceEngine();
  void rgbCallback(const sensor_msgs::Image::ConstPtr& msg);
  //void rgbCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
  //void depthCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void SpinROS();

  bool ImagePairMatches();
  double GetImageTimestamp();

  // ImageSourceEngine
  bool hasMoreImages(void) const;
  void getImages(ITMUChar4Image* rgb, ITMShortImage* raw_depth);
  Vector2i getDepthImageSize(void) const;
  Vector2i getRGBImageSize(void) const;
};

}  // namespace InfiniTAM
