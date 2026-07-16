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

#include <subt_ign/CommonTypes.hh>
#include <subt_ign/protobuf/artifact.pb.h>


bool artifactTypeFromString(const std::string& type_text,
                            subt::ArtifactType& type_enum) {
  // The static initialization assumes single-threading. If multiple threads
  // can call the conversion function at the same time, we can remove `static`
  // and pay the cost of the function building the map at every call. Or do
  // something else about it.
  //
  // In big O notation, std::unordered_map is a better choice than std::map.
  // But this map is so small, that I am not sure if it is the case right here.
  static const std::map<std::string, subt::ArtifactType> conversion = {
    {"TYPE_BACKPACK", subt::ArtifactType::TYPE_BACKPACK},
    {"TYPE_DRILL", subt::ArtifactType::TYPE_DRILL},
    {"TYPE_EXTINGUISHER", subt::ArtifactType::TYPE_EXTINGUISHER},
    {"TYPE_PHONE", subt::ArtifactType::TYPE_PHONE},
    {"TYPE_RESCUE_RANDY", subt::ArtifactType::TYPE_RESCUE_RANDY},
    {"TYPE_VENT", subt::ArtifactType::TYPE_VENT},
    {"TYPE_GAS", subt::ArtifactType::TYPE_GAS},
    {"TYPE_HELMET", subt::ArtifactType::TYPE_HELMET},
    {"TYPE_ROPE", subt::ArtifactType::TYPE_ROPE},
    {"TYPE_CUBE", subt::ArtifactType::TYPE_CUBE},
  };

  const auto type_lookup = conversion.find(type_text);
  if (type_lookup == conversion.end()) {
    return false;
  }
  type_enum = type_lookup->second;
  return true;
}
