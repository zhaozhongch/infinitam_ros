#include "ExternalTrackerEngine.h"

namespace InputSource{

ExternalTrackerEngine::ExternalTrackerEngine(ros::NodeHandle& nh){
    std::string external_pose_topic;
    if(nh.getParam("external_pose_topic",external_pose_topic)){
        ROS_INFO("Camera pose subscribed from external topic %s \n", external_pose_topic.c_str());
    }
    else{
        ROS_INFO("The user wants to use external pose but the topic name is not specified, use default topic name /external_pose");
        external_pose_topic = "/external_pose";
    }
    sub_pose_ = nh.subscribe(external_pose_topic, 1000, &ExternalTrackerEngine::ExternalPoseCallback,this);
}

ExternalTrackerEngine::ExternalTrackerEngine(const ExternalTrackerEngine& ete){}

void ExternalTrackerEngine::ExternalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    ROS_INFO_ONCE("Got external pose, push it into the queue.");
    std::lock_guard<std::mutex> guard(tracker_mutex_);
    pose_queue_.push(*msg);
}

bool ExternalTrackerEngine::GetNewSE3Pose(ORUtils::SE3Pose& pose_oru, double& timestamp_sec){
    if(pose_queue_.empty()){
        ros::Rate r(500);
        ros::Time t0 = ros::Time::now();
        while (ros::ok() && pose_queue_.empty()){
            ros::Time t1 = ros::Time::now();
            if(t1.toSec() - t0.toSec() > 5)
                break;
            r.sleep();
        }

        if(ros::Time::now().toSec() - t0.toSec() > 5){
            ROS_WARN("Didnt get external pose estimation. Timelimit Excess");
            return false;
            //ros::shutdown();
            //exit(0);
        }
    }

    std::lock_guard<std::mutex> guard(tracker_mutex_);
    geometry_msgs::PoseStamped pose = pose_queue_.front();
    pose_queue_.pop();
    tf::Matrix3x3 tfm = tf::Matrix3x3(tf::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w));
    
    ORUtils::Matrix4<float> m;
    //m.m01 means matrix element at row 0 column 1 
    m.m00 = tfm.getColumn(0).getX(); m.m01 = tfm.getColumn(0).getY(); m.m02 = tfm.getColumn(0).getZ(); m.m03 = 0.0;
    m.m10 = tfm.getColumn(1).getX(); m.m11 = tfm.getColumn(1).getY(); m.m12 = tfm.getColumn(1).getZ(); m.m13 = 0.0;
    m.m20 = tfm.getColumn(2).getX(); m.m21 = tfm.getColumn(2).getY(); m.m22 = tfm.getColumn(2).getZ(); m.m23 = 0.0;
    m.m30 = pose.pose.position.x;    m.m31 = pose.pose.position.y;    m.m32= pose.pose.position.z;     m.m33 = 1.0;

    ORUtils::Matrix4<float> invm;
    m.inv(invm);

    pose_oru.SetM(invm);
    timestamp_sec = pose.header.stamp.toSec();
    return true;
}

void ExternalTrackerEngine::StartRosSpinThread(){
    spin_thread_ = new std::thread(&InputSource::ExternalTrackerEngine::SpinROS, this);
}

void ExternalTrackerEngine::SpinROS(){
  ros::Rate r(500);
  printf("Execute ros spin in another thread........................ pose \n");
  while(ros::ok()){
    ros::spinOnce();
    r.sleep();
  }
}

ExternalTrackerEngine::~ExternalTrackerEngine(){
    delete spin_thread_;
}
    
};

