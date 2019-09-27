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
// Hacked original subt_seed_node.cc to report Artifacts only
// coordinates should be via command line. Note, that this is just
// due to only C++ support (all other code is in Python)
//
#include <chrono>

#include <string>
#include <stdlib.h>     /* abs */

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

  /// \brief Callback function for message from other comm clients.
  /// \param[in] _srcAddress The address of the robot who sent the packet.
  /// \param[in] _dstAddress The address of the robot who received the packet.
  /// \param[in] _dstPort The destination port.
  /// \param[in] _data The contents the packet holds.
  private: void CommClientCallback(const std::string &_srcAddress,
                                   const std::string &_dstAddress,
                                   const uint32_t _dstPort,
                                   const std::string &_data);

  /// \brief Communication client.
  private: std::unique_ptr<subt::CommsClient> client;

  /// \brief Name of this robot.
  private: std::string name;


  public: bool ReportArtifact(subt::msgs::Artifact& artifact)
  {
    std::string serializedData;
    if (!artifact.SerializeToString(&serializedData))
    {
//      ROS_ERROR("ReportArtifact(): Error serializing message [%s]",
//          artifact.DebugString().c_str());
      return false;
    }
    else if (!this->client->SendTo(serializedData, subt::kBaseStationName))
    {
//      ROS_ERROR("CommsClient failed to Send serialized data.");
      return false;
    }
    return true;
  }
};

/////////////////////////////////////////////////
Controller::Controller(const std::string &_name)
{
  this->name = _name;
  this->client.reset(new subt::CommsClient(this->name));
  this->client->Bind(&Controller::CommClientCallback, this);
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
//    ROS_INFO("Message Contents[%s]", _data.c_str());
//    fprintf(stdout, "Message Contents[%s]", _data.c_str());
  }

  // Add code to handle communication callbacks.
//  ROS_INFO("Message from [%s] to [%s] on port [%u]:\n [%s]", _srcAddress.c_str(),
//      _dstAddress.c_str(), _dstPort, res.DebugString().c_str());
}



/////////////////////////////////////////////////
int main(int argc, char** argv)
{
  // Create the controller
  Controller controller("X2");

  // Report an artifact
  // Hardcoded to tunnel_circuit_practice_01's exginguisher_3
  subt::msgs::Artifact artifact;
  artifact.set_type(static_cast<uint32_t>(subt::ArtifactType::TYPE_EXTINGUISHER));
  artifact.mutable_pose()->mutable_position()->set_x(-8.1);
  artifact.mutable_pose()->mutable_position()->set_y(37);
  artifact.mutable_pose()->mutable_position()->set_z(0.004);
  controller.ReportArtifact(artifact);

  return 0;
}

