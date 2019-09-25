/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <chrono>
#include <geometry_msgs/Twist.h>
#include <subt_msgs/PoseFromArtifact.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <rosgraph_msgs/Clock.h>

#include <string>

#include <subt_communication_broker/subt_communication_client.h>
#include <subt_ign/CommonTypes.hh>
#include <subt_ign/protobuf/artifact.pb.h>

/// \brief. Example control class, running as a ROS node to control a robot.
class Controller
{
  /// \brief Constructor.
  /// The controller uses the given name as a prefix of its topics, e.g.,
  /// "/x1/cmd_vel" if _name is specified as "x1".
  /// \param[in] _name Name of the robot.
  public: Controller(const std::string &_name);

  /// \brief A function that will be called every loop of the ros spin
  /// cycle.
  public: void Update();

  /// \brief Callback function for message from other comm clients.
  /// \param[in] _srcAddress The address of the robot who sent the packet.
  /// \param[in] _dstAddress The address of the robot who received the packet.
  /// \param[in] _dstPort The destination port.
  /// \param[in] _data The contents the packet holds.
  private: void CommClientCallback(const std::string &_srcAddress,
                                   const std::string &_dstAddress,
                                   const uint32_t _dstPort,
                                   const std::string &_data);

  /// \brief ROS node handler.
  private: ros::NodeHandle n;

  /// \brief publisher to send cmd_vel
  private: ros::Publisher velPub;

  /// \brief Communication client.
  private: std::unique_ptr<subt::CommsClient> client;

  /// \brief Client to request pose from origin.
  private: ros::ServiceClient originClient;

  /// \brief Service to request pose from origin.
  private: subt_msgs::PoseFromArtifact originSrv;

  /// \brief True if robot has arrived at destination.
  public: bool arrived{false};

  /// \brief True if started.
  private: bool started{false};

  /// \brief Last time a comms message to another robot was sent.
  private: std::chrono::time_point<std::chrono::system_clock> lastMsgSentTime;

  /// \brief Name of this robot.
  private: std::string name;

  private: double prev_x{0.0};

  public: bool ReportArtifact(subt::msgs::Artifact& artifact)
  {
    std::string serializedData;
    if (!artifact.SerializeToString(&serializedData))
    {
      ROS_ERROR("ReportArtifact(): Error serializing message [%s]",
          artifact.DebugString().c_str());
      return false;
    }
    else if (!this->client->SendTo(serializedData, subt::kBaseStationName))
    {
      ROS_ERROR("CommsClient failed to Send serialized data.");
      return false;
    }
    return true;
  }
};

/////////////////////////////////////////////////
Controller::Controller(const std::string &_name)
{
  ROS_INFO("Waiting for /clock, /subt/start, and /subt/pose_from_artifact");

  ros::topic::waitForMessage<rosgraph_msgs::Clock>("/clock", this->n);

  // Wait for the start service to be ready.
  ros::service::waitForService("/subt/start", -1);
  ros::service::waitForService("/subt/pose_from_artifact_origin", -1);
  this->name = _name;
  ROS_INFO("Using robot name[%s]\n", this->name.c_str());
}

/////////////////////////////////////////////////
void Controller::CommClientCallback(const std::string &_srcAddress,
                                    const std::string &_dstAddress,
                                    const uint32_t _dstPort,
                                    const std::string &_data)
{
  subt::msgs::ArtifactScore res;
  if (!res.ParseFromString(_data))
  {
    ROS_INFO("Message Contents[%s]", _data.c_str());
  }

  // Add code to handle communication callbacks.
  ROS_INFO("Message from [%s] to [%s] on port [%u]:\n [%s]", _srcAddress.c_str(),
      _dstAddress.c_str(), _dstPort, res.DebugString().c_str());
}

/////////////////////////////////////////////////
void Controller::Update()
{
  if (!this->started)
  {
    // Send start signal
    std_srvs::SetBool::Request req;
    std_srvs::SetBool::Response res;
    req.data = true;
    if (!ros::service::call("/subt/start", req, res))
    {
      ROS_ERROR("Unable to send start signal.");
    }
    else
    {
      ROS_INFO("Sent start signal.");
      this->started = true;
    }

    if (this->started)
    {
      // Create subt communication client
      this->client.reset(new subt::CommsClient(this->name));
      this->client->Bind(&Controller::CommClientCallback, this);

      // Create a cmd_vel publisher to control a vehicle.
      this->velPub = this->n.advertise<geometry_msgs::Twist>(
          this->name + "/cmd_vel", 1);

      // Create a cmd_vel publisher to control a vehicle.
      this->originClient = this->n.serviceClient<subt_msgs::PoseFromArtifact>(
          "/subt/pose_from_artifact_origin");
      this->originSrv.request.robot_name.data = this->name;
    }
    else
      return;
  }

  if (this->arrived)
    return;

  bool call = this->originClient.call(this->originSrv);
  // Query current robot position w.r.t. entrance
  if (!call || !this->originSrv.response.success)
  {
    ROS_ERROR("Failed to call pose_from_artifact_origin service, \
robot may not exist, be outside staging area, or the service is \
not available.");

    // Stop robot
    geometry_msgs::Twist msg;
    this->velPub.publish(msg);
    return;
  }

  auto pose = this->originSrv.response.pose.pose;

  // Simple example for robot to go to entrance
  geometry_msgs::Twist msg;

  // Distance to goal
  double dist = pose.position.x * pose.position.x +
    pose.position.y * pose.position.y;

  if(prev_x != pose.position.x)
    ROS_INFO_STREAM("MD robot pose " << pose.position.x << " " << pose.position.y << " dist=" << dist);
  prev_x = pose.position.x;

  // Arrived
  if (dist < 0.3 || pose.position.x >= -0.3)
  {
    msg.linear.x = 0;
    msg.angular.z = 0;
    this->arrived = true;
    ROS_INFO("Arrived at entrance!");

    // Report an artifact
    // Hardcoded to tunnel_circuit_practice_01's exginguisher_3
    subt::msgs::Artifact artifact;
    artifact.set_type(static_cast<uint32_t>(subt::ArtifactType::TYPE_EXTINGUISHER));
    artifact.mutable_pose()->mutable_position()->set_x(-8.1);
    artifact.mutable_pose()->mutable_position()->set_y(37);
    artifact.mutable_pose()->mutable_position()->set_z(0.004);

//    this->ReportArtifact(artifact);
    /*
    std::string serializedData;
    if (!artifact.SerializeToString(&serializedData))
    {
      ROS_ERROR("ReportArtifact(): Error serializing message [%s]",
          artifact.DebugString().c_str());
    }
    else if (!this->client->SendTo(serializedData, subt::kBaseStationName))
    {
      ROS_ERROR("CommsClient failed to Send serialized data.");
    }*/
  }
}

/////////////////////////////////////////////////
int main(int argc, char** argv)
{
  // Initialize ros
  ros::init(argc, argv, argv[1]);

  ROS_INFO("Starting seed competitor\n");
  std::string name;

  // Get the name of the robot based on the name of the "cmd_vel" topic if
  // the name was not passed in as an argument.
  if (argc < 2 || std::strlen(argv[1]) == 0)
  {
    while (name.empty())
    {
      ros::master::V_TopicInfo masterTopics;
      ros::master::getTopics(masterTopics);

      for (ros::master::V_TopicInfo::iterator it = masterTopics.begin();
          it != masterTopics.end(); ++it)
      {
        const ros::master::TopicInfo &info = *it;
        if (info.name.find("battery_state") != std::string::npos)
        {
          int rpos = info.name.rfind("/");
          name = info.name.substr(1, rpos - 1);
        }
      }
      if (name.empty())
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  // Otherwise use the name provided as an argument.
  else
  {
    name = argv[1];
  }

  // Create the controller
  Controller controller(name);

  // This sample code iteratively calls Controller::Update. This is just an
  // example. You can write your controller using alternative methods.
  // To get started with ROS visit: http://wiki.ros.org/ROS/Tutorials
  ros::Rate loop_rate(10);
  while (ros::ok() && !controller.arrived)
  {
    controller.Update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  char *path = "/home/developer/subt_solution/osgar/call_base.txt";
  double offset_x, offset_y, offset_z;

  offset_x = -6.0;
  offset_y = 5.0;
  offset_z = 0.0;

  int i;
  ros::Rate r(1);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
    ROS_INFO("MD Test\n");
    FILE *fd = fopen(path, "r");
    if(fd != NULL)
    {
      ROS_INFO("MD SendToBaseStation\n");
      double x, y, z;
      char buf[256];
      bool ret = false;
      while(fscanf(fd, "%s %lf %lf %lf", buf, &x, &y, &z) == 4)
      {
        ROS_INFO_STREAM("MD artf" << buf);
        ROS_INFO_STREAM("MD x" << x);
        ROS_INFO_STREAM("MD y" << y);
        ROS_INFO_STREAM("MD z" << z);

//        pose.mutable_position()->set_x(x + offset_x);
//        pose.mutable_position()->set_y(y + offset_y);
//        pose.mutable_position()->set_z(z + offset_z);
        
        subt::msgs::Artifact artifact;
//        artifact.set_type(static_cast<uint32_t>(subt::ArtifactType::TYPE_EXTINGUISHER));
        artifact.mutable_pose()->mutable_position()->set_x(x + offset_x);
        artifact.mutable_pose()->mutable_position()->set_y(y + offset_y);
        artifact.mutable_pose()->mutable_position()->set_z(z + offset_z);

        subt::ArtifactType type;
        if(strcmp(buf, "TYPE_BACKPACK") == 0)
        {
          type = subt::ArtifactType::TYPE_BACKPACK;
        }
        if(strcmp(buf, "TYPE_DUCT") == 0)
        {
          type = subt::ArtifactType::TYPE_DUCT;
        }
        if(strcmp(buf, "TYPE_DRILL") == 0)
        {
          type = subt::ArtifactType::TYPE_DRILL;
        }
        if(strcmp(buf, "TYPE_ELECTRICAL_BOX") == 0)
        {
          type = subt::ArtifactType::TYPE_ELECTRICAL_BOX;
        }
        if(strcmp(buf, "TYPE_EXTINGUISHER") == 0)
        {
          type = subt::ArtifactType::TYPE_EXTINGUISHER;
        }
        if(strcmp(buf, "TYPE_PHONE") == 0)
        {
          type = subt::ArtifactType::TYPE_PHONE;
        }
        if(strcmp(buf, "TYPE_RADIO") == 0)
        {
          type = subt::ArtifactType::TYPE_RADIO;
        }
        if(strcmp(buf, "TYPE_RESCUE_RANDY") == 0)
        {
          type = subt::ArtifactType::TYPE_RESCUE_RANDY;
        }
        if(strcmp(buf, "TYPE_TOOLBOX") == 0)
        {
          type = subt::ArtifactType::TYPE_TOOLBOX;
        }
        if(strcmp(buf, "TYPE_VALVE") == 0)
        {
          type = subt::ArtifactType::TYPE_VALVE;
        }

        artifact.set_type(static_cast<uint32_t>(type));

        ROS_INFO_STREAM("MD enum" << static_cast<uint32_t>(type));

        ret |= controller.ReportArtifact(artifact);
        if(ret)
        {
          ROS_INFO("MD SUCCESS\n");
        }
        else
          ROS_INFO("MD FAILURE\n");
      }
      if(ret)
      {
        break;  // at least some artifacts sucessfully reported
      }
    }
    i++;
  }
  ros::spin();
  return 0;
}
