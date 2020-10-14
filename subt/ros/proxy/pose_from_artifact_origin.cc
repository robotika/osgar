#include <cstring>
#include <cstdio>
#include <string>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <subt_msgs/PoseFromArtifact.h>
#include <ros/ros.h>

class PoseFromArtifactOrigin
{
  public:
    PoseFromArtifactOrigin(const std::string& robot_name, const std::string& frame_id);

  private:
    ros::NodeHandle node_handle;
    ros::Timer timer;

    ros::ServiceClient origin_client;
    subt_msgs::PoseFromArtifact origin_rpc;

    ros::Publisher pose_publisher;
    geometry_msgs::PoseWithCovarianceStamped published_pose;
   
    void Update(const ros::TimerEvent&);
};

PoseFromArtifactOrigin::PoseFromArtifactOrigin(const std::string& robot_name, const std::string& frame_id)
{
  origin_client = node_handle.serviceClient<subt_msgs::PoseFromArtifact>("/subt/pose_from_artifact_origin", true);
  origin_rpc.request.robot_name.data = robot_name;

  timer = node_handle.createTimer(ros::Duration(0.02), &PoseFromArtifactOrigin::Update, this);
  pose_publisher = node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose_from_origin", 100);
  published_pose.header.frame_id = frame_id;
  published_pose.header.seq = 0;

  ros::service::waitForService("/subt/pose_from_artifact_origin", -1);
}

void PoseFromArtifactOrigin::Update(const ros::TimerEvent&)
{
  const bool success = origin_client.call(origin_rpc);
  if (success && origin_rpc.response.success)
  {
    published_pose.pose.pose = origin_rpc.response.pose.pose;
    published_pose.header.stamp = ros::Time::now();
    pose_publisher.publish(published_pose);
    ++published_pose.header.seq;
  }
}

int main(int argc, char** argv)
{
  if (argc < 3 || std::strlen(argv[1]) == 0 || std::strlen(argv[2]) == 0) {
    std::cerr << "Need robot name and frame id arguments." << std::endl;
    return -1;
  }

  std::string robot_name = argv[1];
  std::string frame_id = argv[2];
  ros::init(argc, argv, robot_name);

  ROS_INFO_STREAM("Starting pose_from_artifact_origin " << robot_name << ", " << frame_id);

  PoseFromArtifactOrigin pose_from_artifact_origin(robot_name, frame_id);
  ros::spin();
  return 0;
}
