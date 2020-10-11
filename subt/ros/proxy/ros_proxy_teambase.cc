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
// TEAMBASE attempt based on subt_seed_node.cc
// Goal:
//   - handle ROS communication
//   - send speed commands
//   - redirect clock messages to Python3 (via ZeroMQ)

#include <chrono>
#include <thread>
#include <fstream>
#include <mutex>

#include <subt_msgs/PoseFromArtifact.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

#include <string>
#include <stdlib.h>     /* abs */

#include <subt_communication_broker/subt_communication_client.h>
#include <subt_ign/CommonTypes.hh>
#include <subt_ign/protobuf/artifact.pb.h>

// PROXY PART
#include <zmq.h>
#include <assert.h>

const uint32_t BROADCAST_PORT = 4142u; // default is 4100 and collides with artifact messages

int g_countClock = 0;
uint32_t g_clockPrevSec = 0;

int g_countReceives = 0;

void *g_context;
void *g_responder;

std::mutex g_zmq_mutex;

std::unique_ptr<subt::CommsClient> g_client;


void initZeroMQ()
{
  g_context = zmq_ctx_new ();
  g_responder = zmq_socket (g_context, ZMQ_PUSH);  // use "Pipeline pattern" to send all data to Python3
  int rc = zmq_bind (g_responder, "tcp://*:5555");
  if (rc != 0) {
    ROS_ERROR("zmq_bind failed");
    exit(1);
  }
}

// int zmq_send (void *socket, void *buf, size_t len, int flags);
int protected_zmq_send(void *socket, const void *buf, size_t len, int flags)
{
  const std::lock_guard<std::mutex> lock(g_zmq_mutex);
  return zmq_send(socket, buf, len, flags);
}

void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg)
{
  ros::SerializedMessage sm = ros::serialization::serializeMessage(*msg);
  if(g_clockPrevSec != msg->clock.sec)
  {
    protected_zmq_send(g_responder, sm.buf.get(), sm.num_bytes, 0);
    g_clockPrevSec = msg->clock.sec;
  }
  if(g_countClock % 1000 == 0)
    ROS_INFO("received Clock %d ", g_countClock);
  g_countClock++;
}

void sendReceivedMessage(const std::string &srcAddress, const std::string &data)
{
  char buf[10000];  // the limit for messages is 4k?
  int size = sprintf(buf, "radio %s %s", srcAddress.c_str(), data.c_str());
  protected_zmq_send(g_responder, buf, size, 0);
}

void commClientCallback(const std::string &_srcAddress,
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
  sendReceivedMessage(_srcAddress, _data);
}

bool reportArtifact(subt::msgs::Artifact& artifact)
{
  if (g_client == 0) {
      ROS_INFO_STREAM("no client");
      return false;
  }

  std::string serializedData;
  if (!artifact.SerializeToString(&serializedData))
  {
    ROS_ERROR("ReportArtifact(): Error serializing message [%s]",
        artifact.DebugString().c_str());
    return false;
  }
  else if (g_client->SendTo(serializedData, subt::kBaseStationName))
  {
    ROS_ERROR("CommsClient failed to Send serialized data.");
    return false;
  }
  return true;
}

bool broadcast(std::string& serializedData)
{
  if (g_client == 0) {
      ROS_INFO_STREAM("no client");
      return false;
  }
  if (g_client->SendTo(serializedData, subt::kBroadcast, BROADCAST_PORT))
  {
    ROS_ERROR("CommsClient failed to broadcast serialized data.");
    return false;
  }
  return true;
}

bool parseArtf(char *input_str, subt::msgs::Artifact& artifact)
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
        if(strcmp(buf, "TYPE_VENT") == 0)
        {
          type = subt::ArtifactType::TYPE_VENT;
        }
        if(strcmp(buf, "TYPE_GAS") == 0)
        {
          type = subt::ArtifactType::TYPE_GAS;
        }
        if(strcmp(buf, "TYPE_HELMET") == 0)
        {
          type = subt::ArtifactType::TYPE_HELMET;
        }
        if(strcmp(buf, "TYPE_ROPE") == 0)
        {
          type = subt::ArtifactType::TYPE_ROPE;
        }

        artifact.set_type(static_cast<uint32_t>(type));

        ROS_INFO_STREAM("MD enum = " << static_cast<uint32_t>(type));
        return true;
  }
  return false;
}

/////////////////////////////////////////////////
void receiveZmqThread()
{
  void * contextIn = zmq_ctx_new ();
  void * requester = zmq_socket (contextIn, ZMQ_PULL);  // use "Pipeline pattern" to receive all data from Python3
  int recv_timeout_ms = 300;
  int linger_timeout_ms = 100;
  zmq_setsockopt(requester, ZMQ_RCVTIMEO, &recv_timeout_ms, sizeof(recv_timeout_ms));
  zmq_setsockopt(requester, ZMQ_LINGER, &linger_timeout_ms, sizeof(linger_timeout_ms));
  int rc = zmq_bind (requester, "tcp://*:5556");
  if (rc != 0) {
    ROS_ERROR("zmq_bind for receiver failed, exiting receiver thread");
    return;
  }

  ROS_INFO("zmq receive thread started");

  char buffer[10000];
  int size;

  while (ros::ok()) {
    size = zmq_recv(requester, buffer, sizeof(buffer)-1, 0);

    if (size < 0) {
      continue;
    }

    buffer[size] = 0;

    if(g_countReceives % 100 == 0) {
        ROS_INFO("receiveZmq count %d", g_countReceives);
    }
    g_countReceives++;

    if(strncmp(buffer, "stdout ", 7) == 0)
    {
      ROS_INFO("Python3: %s", buffer);
    }
    else if(strncmp(buffer, "artf ", 5) == 0)
    {
      subt::msgs::Artifact artifact;
      ROS_INFO("artf: %s", buffer);
      if(parseArtf(buffer + 5, artifact)) // skip initial prefix "artf "
      {
        if(reportArtifact(artifact))
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
    else if(strncmp(buffer, "broadcast ", 10) == 0)
    {
        std::string content(buffer + 10);
        ROS_INFO_STREAM("BROADCAST RECEIVED " << content);
        if(broadcast(content))
        {
          ROS_INFO("MD BROADCAST SUCCESS\n");
        }
        else
          ROS_INFO("MD BROADCAST FAILURE\n");
    }
  }
  ROS_INFO("zmq receive thread finished");
  zmq_close(requester);
  zmq_term(contextIn);
}

/////////////////////////////////////////////////
int main(int argc, char** argv)
{
  if (argc < 2 || std::strlen(argv[1]) == 0) {
    std::cerr << "Need robot name argument." << std::endl;
    return -1;
  }

  // Initialize ros
  std::string robot_name = argv[1];
  ros::init(argc, argv, robot_name);

  ROS_INFO_STREAM("Starting robotika TEAMBASE solution for robot " << robot_name);

  initZeroMQ();

  g_client.reset(new subt::CommsClient(robot_name));
  g_client->Bind(&commClientCallback, "", BROADCAST_PORT); // from other robots
  g_client->Bind(&commClientCallback);                     // from darpa basestation

  ros::NodeHandle n;
  ros::Subscriber subClock;
  subClock  = n.subscribe("/clock", 1000, clockCallback);

  ROS_INFO("Waiting for /clock, /subt/start");
  ros::topic::waitForMessage<rosgraph_msgs::Clock>("/clock", n);

  // Wait for the start service to be ready.
  ros::service::waitForService("/subt/start", -1);

  ROS_INFO_STREAM("Sleeping 1 simulated second to let simulation start up");
  ros::Duration(1).sleep();
  ROS_INFO_STREAM("Simulation hopefully up and running");

  std::thread receiveZmq;
  receiveZmq = std::thread(receiveZmqThread);

  ros::spin();
  return 0;
}
