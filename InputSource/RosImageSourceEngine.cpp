// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "RosImageSourceEngine.h"

#include <cstdio>
#include <stdexcept>
#include <string>

#include "../ORUtils/FileUtils.h"


//#ifdef COMPILE_WITH_Ros

namespace InputSource {

RosImageSourceEngine::RosImageSourceEngine(ros::NodeHandle& nh,
                                           const char*& calibration_filename)
    : BaseImageSourceEngine(calibration_filename)
 {

  std::string rgb_image_topic;
  std::string depth_image_topic;

  if(nh.getParam("rgb_image_topic", rgb_image_topic)){
    ROS_INFO("Get rgb image topic name %s \n",rgb_image_topic.c_str());
  }
  else{
    ROS_INFO("No rgb image topic name, use default /camera/rgb/image_color");
    rgb_image_topic = "/camera/rgb/image_color";
  }

  if(nh.getParam("depth_image_topic", depth_image_topic)){
    ROS_INFO("Get depth image topic name %s \n",depth_image_topic.c_str());
  }
  else{
    ROS_INFO("No depth image topic name, use default /camera/depth/image");
    depth_image_topic = "/camera/depth/image";
  }

  if(nh.getParam("depth_scale", depth_scale_)){
    ROS_INFO("Get depth scale %f. This should be the inverse of the first `affine` parameter in the calibration file \n", depth_scale_);
  }
  else{
    ROS_INFO("Use default depth scale 1.0");
  }

  // Get images from ROS topic.
  rgb_sub_ = nh.subscribe(rgb_image_topic, 10,
                                    &RosImageSourceEngine::rgbCallback,
                                    this);

  depth_sub_ = nh.subscribe(depth_image_topic, 10,
                                      &RosImageSourceEngine::depthCallback,
                                      this);

  auto calib = this->getCalib();
  image_size_depth_ = calib.intrinsics_d.imgSize;
  image_size_rgb_   = calib.intrinsics_rgb.imgSize;

  spin_thread_ = new std::thread(&InputSource::RosImageSourceEngine::SpinROS,this);

}

RosImageSourceEngine::~RosImageSourceEngine() {
  delete spin_thread_;
}

void RosImageSourceEngine::rgbCallback(
    const sensor_msgs::Image::ConstPtr& msg) {
  ROS_INFO_ONCE("Got rgb raw image, push it into the queue.");
  cv_bridge::CvImagePtr cv_rgb_image;
  if(msg->encoding == sensor_msgs::image_encodings::RGB8)
    cv_rgb_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  else if(msg->encoding == sensor_msgs::image_encodings::BGR8)
    cv_rgb_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  else{
    ROS_FATAL("unsupported rgb image encodings");
    ros::shutdown();
  }
  std::lock_guard<std::mutex> guard(rgb_mutex_);
  rgb_queue_.push(cv_rgb_image);
}

void RosImageSourceEngine::depthCallback(
    const sensor_msgs::Image::ConstPtr& msg) {
  ROS_INFO_ONCE("Got depth raw image. Push it into the queue");

  depth_msg_time_stamp_ = msg->header.stamp;
  cv_bridge::CvImagePtr cv_depth_image;

  if(msg->encoding != sensor_msgs::image_encodings::TYPE_32FC1 &&  msg->encoding != sensor_msgs::image_encodings::TYPE_16UC1){
    ROS_FATAL("unsupported depth image encodings");
    ros::shutdown();
  }
  cv_depth_image = cv_bridge::toCvCopy(msg, msg->encoding);
  // If the image has 32FC1. Infinitam needs 16UCI.
  if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    (cv_depth_image->image)
        .convertTo(cv_depth_image->image, CV_16UC1, depth_scale_);
  }

  std::lock_guard<std::mutex> guard(depth_mutex_);
  depth_queue_.push(cv_depth_image);
}


//convert ros image to infinitam version
void RosImageSourceEngine::getImages(ITMUChar4Image* rgb_image,
ITMShortImage* raw_depth_image) {

  short* raw_depth_infinitam = raw_depth_image->GetData(MEMORYDEVICE_CPU);
  const uint16_t* depth_frame = reinterpret_cast<const uint16_t*>(cv_depth_image_->image.data);
  constexpr size_t depth_pixel_size = sizeof(uint16_t);
	static_assert(2 == depth_pixel_size, "sizeof(depth pixel) must equal 2");

	std::memcpy(raw_depth_infinitam, depth_frame, depth_pixel_size * raw_depth_image->noDims.x * raw_depth_image->noDims.y);

  Vector4u* rgb_infinitam = rgb_image->GetData(MEMORYDEVICE_CPU);

  cv::Size rgb_size = cv_rgb_image_->image.size();
  uint rgb_rows = rgb_size.height;
  uint rgb_cols = rgb_size.width;
  for (size_t i = 0; i < 3 * rgb_rows * rgb_cols; i += 3) {
    Vector4u pixel_value;
    pixel_value.r = cv_rgb_image_->image.data[i];
    pixel_value.g = cv_rgb_image_->image.data[i + 1];
    pixel_value.b = cv_rgb_image_->image.data[i + 2];
    pixel_value.w = 255;
    rgb_infinitam[i / 3] = pixel_value;
  }

}

bool RosImageSourceEngine::ImagePairMatches(){
  double t1, t2, th = 0.033;
  
  std::lock_guard<std::mutex> rgb_guard(rgb_mutex_);
  std::lock_guard<std::mutex> depth_guard(depth_mutex_);

  cv_depth_image_ = depth_queue_.front();
  cv_rgb_image_ = rgb_queue_.front();

  depth_queue_.pop();
  rgb_queue_.pop();

  t1 = cv_depth_image_->header.stamp.toSec();
  t2 = cv_rgb_image_->header.stamp.toSec();

  if(t1 - t2 > th){
    while(ros::ok() && !rgb_queue_.empty() && t1 - t2 > th){
      cv_rgb_image_ = rgb_queue_.front();
      rgb_queue_.pop();
      t2 = cv_rgb_image_->header.stamp.toSec();
    }

    if(rgb_queue_.empty() && t1 - t2 > th){
      ROS_WARN("didnt find a good image pair with timestamp match");
      return false;
    }
    
    if(t2 - t1 > th){
      ROS_WARN("depth and rgb images are not synchronized, yoy should try to use a proper device that has synchronized image");
      return false;
    }
  }
  else if(t2 - t1 > th){
    while(ros::ok() && !depth_queue_.empty() && t2 - t1 > th){
      cv_depth_image_ = depth_queue_.front();
      depth_queue_.pop();
      t1 = cv_depth_image_->header.stamp.toSec();
    }

    if(depth_queue_.empty() && t2 - t1 > th){
      ROS_WARN("didnt find a good image pair with timestamp match");
      return false;
    }

    if(t1 - t2 > th){
      ROS_WARN("depth and rgb images are not synchronized, you should try to use a proper device that has synchronized image");
      return false;
    }
  }

  timestamp_sec_ = t1;

  return true;
}


bool RosImageSourceEngine::hasMoreImages(void) const{
  if(depth_queue_.empty()|| rgb_queue_.empty())
    return false;

  return true;
}

double RosImageSourceEngine::GetImageTimestamp(){
  return timestamp_sec_;
}

Vector2i RosImageSourceEngine::getDepthImageSize(void) const {
  return image_size_depth_;
}
Vector2i RosImageSourceEngine::getRGBImageSize(void) const {
  return image_size_rgb_;
}

void RosImageSourceEngine::SpinROS(){
  ros::Rate r(500);
  printf("Execute ros spin in another thread........................ image \n");
  while(ros::ok()){
    ros::spinOnce();
    r.sleep();
  }
}

}  // namespace InfiniTAM

