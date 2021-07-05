#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include "../ORUtils/SE3Pose.h"
#include <mutex>
#include <queue>
#include <thread>

namespace InputSource {

class ExternalTrackerEngine{
private:
    std::mutex tracker_mutex_;

    std::thread* spin_thread_;
    
    std::queue<geometry_msgs::PoseStamped> pose_queue_;
public:
ros::Subscriber sub_pose_;
    ExternalTrackerEngine(ros::NodeHandle& nh);
    ExternalTrackerEngine(const ExternalTrackerEngine& ete);
    //ROS spin start in ROSImageSourceEngine
    void ExternalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    bool GetNewSE3Pose(ORUtils::SE3Pose& pose_oru, double& timestamp_sec);
    void StartRosSpinThread();
    void SpinROS();
    ~ExternalTrackerEngine();
};
}