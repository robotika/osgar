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
// Another attempt based on subt_seed_node.cc
// Goal:
//   - handle all artifacts, ROS communication
//   - send speed commands
//   - redirect IMU, Odom, Scan and Image messages to Python3 (via ZeroMQ?)

#include <chrono>
#include <geometry_msgs/Twist.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CompressedImage.h>
#include <nav_msgs/Odometry.h>

#include <subt_msgs/PoseFromArtifact.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/String.h>

#include <string>
#include <stdlib.h>     /* abs */

#include <subt_communication_broker/subt_communication_client.h>
#include <subt_ign/CommonTypes.hh>
#include <subt_ign/protobuf/artifact.pb.h>

// PROXY PART
#include <zmq.h>
#include <assert.h>

int g_countClock = 0;
int g_countImu = 0;
int g_countScan = 0;
int g_countImage = 0;
int g_countOdom = 0;

void *g_context;
void *g_responder;

void *g_contextIn;
void *g_requester;

void initZeroMQ()
{
  g_context = zmq_ctx_new ();
  g_responder = zmq_socket (g_context, ZMQ_PUSH);  // use "Pipeline pattern" to send all data to Python3
  int rc = zmq_bind (g_responder, "tcp://*:5555");
  assert (rc == 0);

  // received
  g_contextIn = zmq_ctx_new ();
  g_requester = zmq_socket (g_contextIn, ZMQ_PULL);  // use "Pipeline pattern" to receive all data to Python3
  rc = zmq_bind (g_requester, "tcp://*:5556");
  assert (rc == 0);
}

void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg)
{
  ros::SerializedMessage sm = ros::serialization::serializeMessage(*msg);
  zmq_send(g_responder, sm.buf.get(), sm.num_bytes, 0);
  if(g_countClock % 1000 == 0)
    ROS_INFO("received Clock %d ", g_countClock);
  g_countClock++;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ros::SerializedMessage sm = ros::serialization::serializeMessage(*msg);
  zmq_send(g_responder, sm.buf.get(), sm.num_bytes, 0);
  if(g_countImu % 100 == 0)
    ROS_INFO("received Imu %d ", g_countImu);
  g_countImu++;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ros::SerializedMessage sm = ros::serialization::serializeMessage(*msg);
  zmq_send(g_responder, sm.buf.get(), sm.num_bytes, 0);
  if(g_countScan % 100 == 0)
    ROS_INFO("received Scan %d", g_countScan);
  g_countScan++;
}

void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
  ros::SerializedMessage sm = ros::serialization::serializeMessage(*msg);
  zmq_send(g_responder, sm.buf.get(), sm.num_bytes, 0);
  if(g_countImage % 100 == 0)
    ROS_INFO("received Image %d", g_countImage);
  g_countImage++;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ros::SerializedMessage sm = ros::serialization::serializeMessage(*msg);
  zmq_send(g_responder, sm.buf.get(), sm.num_bytes, 0);
  if(g_countOdom % 100 == 0)
    ROS_INFO("received Odom %d", g_countOdom);
  g_countOdom++;
}

void sendOrigin(std::string& name, double x, double y, double z,
                double qx, double qy, double qz, double qw)
{
  char buf[1000];
  int size = sprintf(buf, "origin %s %lf %lf %lf  %lf %lf %lf %lf", name.c_str(), x, y, z, qx, qy, qz, qw);
  zmq_send(g_responder, buf, size, 0);
}

void sendOriginError(std::string& name)
{
  char buf[1000];
  int size = sprintf(buf, "origin %s ERROR", name.c_str());
  zmq_send(g_responder, buf, size, 0);
}


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
  public: ros::Publisher velPub;

  /// \brief publisher to send logging data
  public: ros::Publisher robotDataPub;

  /// \brief Communication client.
  private: std::unique_ptr<subt::CommsClient> client;

  /// \brief Client to request pose from origin.
  ros::ServiceClient originClient;

  /// \brief Service to request pose from origin.
  subt_msgs::PoseFromArtifact originSrv;

  ros::Subscriber subClock;
  ros::Subscriber subImu;
  ros::Subscriber subScan;
  ros::Subscriber subImage;
  ros::Subscriber subOdom;

  /// \brief True if robot has arrived at destination.
  public: bool arrived{false};

  /// \brief True if started.
  private: bool started{false};

  /// \brief Last time a comms message to another robot was sent.
  private: std::chrono::time_point<std::chrono::system_clock> lastMsgSentTime;

  /// \brief Name of this robot.
  private: std::string name;

  private: std::string logFilename;
  long int log_offset{-1};

  private: double prev_dist2{0.0};

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

  public: bool getSpeedCmd(geometry_msgs::Twist& msg);
  public: bool parseArtf(char *input_str, subt::msgs::Artifact& artifact);
  public: void sendLogPart();
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

      // Create data logging publisher
      // https://bitbucket.org/osrf/subt/pull-requests/329/support-recording-up-to-2gb-of-custom-data/diff
      this->robotDataPub = this->n.advertise<std_msgs::String>(
          "/robot_data", 1);

      this->subClock  = n.subscribe("/clock", 1000, clockCallback);
      this->subImu  = n.subscribe(this->name + "/imu/data", 1000, imuCallback);
      this->subScan = n.subscribe(this->name + "/front_scan", 1000, scanCallback);
      this->subImage = n.subscribe(this->name + "/front/image_raw/compressed", 1000, imageCallback);
      this->subOdom = n.subscribe(this->name + "/odom", 1000, odomCallback);

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

    sendOriginError(this->name);
    this->arrived = true; // give up ... is it good idea?

    // Stop robot
    geometry_msgs::Twist msg;
    this->velPub.publish(msg);
    return;
  }

  auto pose = this->originSrv.response.pose.pose;
  // send position to Python3 code
  sendOrigin(this->name, pose.position.x, pose.position.y, pose.position.z,
             pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

  // Simple example for robot to go to entrance
  geometry_msgs::Twist msg;
  while(this->getSpeedCmd(msg))
  {
    this->velPub.publish(msg);
  }

  // Distance to goal
  double dist = pose.position.x * pose.position.x +
    pose.position.y * pose.position.y;

  if(abs(prev_dist2 - dist) > 1.0)
  {
    ROS_INFO_STREAM("MD robot pose " << pose.position.x << " " << pose.position.y << " dist=" << dist);
    prev_dist2 = dist;
  }

  // Arrived
  if (dist < 0.3 || pose.position.x >= -0.3)
  {
    this->arrived = true;
    ROS_INFO("Arrived at entrance!");
  }
}


void Controller::sendLogPart()
{
   // the ROS logger availability is currently not known
   // see https://bitbucket.org/osrf/subt/issues/276/when-is-ros-bag-recorder-ready
  if (g_countClock < 1000)
    return;

  // Test empty log
  // based on Nate example:
  //   https://bitbucket.org/osrf/subt_seed/branch/log_sensor_msgs#diff
  std_msgs::String log;
  std::ostringstream stream;
  ros::Time currentTime = ros::Time::now();
  stream << currentTime.sec + currentTime.nsec*1e-9 << ": Hello " << this->name;
  log.data = stream.str().c_str();
  this->robotDataPub.publish(log);

  if (this->log_offset >= 0)
  {
    FILE *fd = fopen(this->logFilename.c_str(), "rb");
    if(fd != NULL)
    {
      char buf[1000000];
      fseek(fd, this->log_offset, SEEK_SET);
      size_t size = fread(buf, 1, 1000000, fd);
      fclose(fd);
      this->log_offset += size;
      std::string s;
      size_t i;
      for(i = 0; i < size; i++)
        s += buf[i];
      log.data = s;
      this->robotDataPub.publish(log);
    }
    else
    {
      ROS_INFO_STREAM("Error opening: " << this->logFilename);
    }
  }
}

/////////////////////////////////////////////////
bool Controller::parseArtf(char *input_str, subt::msgs::Artifact& artifact)
{
  double x, y, z;
  char buf[256];

  if(sscanf(input_str, "%s %lf %lf %lf", buf, &x, &y, &z) == 4)
  {
        ROS_INFO_STREAM("MD artf = " << buf);
        ROS_INFO_STREAM("MD x = " << x);
        ROS_INFO_STREAM("MD y = " << y);
        ROS_INFO_STREAM("MD z = " << z);

        artifact.mutable_pose()->mutable_position()->set_x(x);
        artifact.mutable_pose()->mutable_position()->set_y(y);
        artifact.mutable_pose()->mutable_position()->set_z(z);

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

        ROS_INFO_STREAM("MD enum = " << static_cast<uint32_t>(type));
        return true;
  }
  return false;
}

/////////////////////////////////////////////////
bool Controller::getSpeedCmd(geometry_msgs::Twist& msg)
{ 
  char buffer[10000];
  int size;
  while((size=zmq_recv(g_requester, buffer, 10000, ZMQ_DONTWAIT)) > 0)
  {
    if(strncmp(buffer, "stdout ", 7) == 0)
    {
      buffer[size] = 0;
      ROS_INFO("Python3: %s", buffer);
    }
    else if(strncmp(buffer, "request_origin", 14) == 0)
    {
      this->arrived = false;  // re-enable origin query
    }
    else if(strncmp(buffer, "artf ", 5) == 0)
    {
      subt::msgs::Artifact artifact;
      buffer[size] = 0;
      ROS_INFO("artf: %s", buffer);
      if(parseArtf(buffer + 5, artifact)) // skip initial prefix "artf "
      {
        if(this->ReportArtifact(artifact))
        {
          ROS_INFO("MD SUCCESS\n");
        }
        else
          ROS_INFO("MD FAILURE\n");
      }
      else
      {
        ROS_INFO("ERROR! - failed to parse received artifact info");
      }
    }
    else if(strncmp(buffer, "file ", 5) == 0)
    {
      buffer[size] = 0;
      ROS_INFO("FILE: %s", buffer);
      this->logFilename = buffer + 5;
      this->log_offset = 0;
    }
    else
    {
      double speed, angular_speed;
      int c = sscanf(buffer, "cmd_vel %lf %lf", &speed, &angular_speed);
      if(c == 2)
      {
        msg.linear.x = speed;
        msg.angular.z = angular_speed;
        return true;
      }
      else
      {
        ROS_INFO_STREAM("MD bad parsing" << c << " " << buffer);
        break;
      }
    }
  }
  return false;
}

/////////////////////////////////////////////////
int main(int argc, char** argv)
{
  // init ROS proxy server
  initZeroMQ();

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
  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    while (ros::ok() && !controller.arrived)
    {
      controller.Update();
      ros::spinOnce();
      loop_rate.sleep();
      controller.sendLogPart();
    }

    while (ros::ok() && controller.arrived)
    {
      geometry_msgs::Twist msg;
      while(controller.getSpeedCmd(msg))
      {
        controller.velPub.publish(msg);
      }
      ros::spinOnce();
      loop_rate.sleep();
      controller.sendLogPart();
    }
  }
  ros::spin();
  return 0;
}
