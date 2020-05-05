#!/usr/bin/env bash

#
# Copyright (C) 2018 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

# Builds a Docker image.

if [ $# -eq 0 ]
then
    echo "Usage: $0 directory-name"
    exit 1
fi

# get path to current directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

if [ ! -d $DIR/$1 ]
then
  echo "image-name must be a directory in the same folder as this script"
  exit 2
fi

user_id=$(id -u)
image_name=$(basename $1)
image_plus_tag=$image_name:$(date +%Y_%m_%d_%H%M)
# mercurial adds a + symbol if there are uncomitted changes in the repo
# that will break docker tag syntax
#hg_id=$(hg id -i | tr -d '+')

shift


docker build --rm -t $image_plus_tag -f $DIR/$image_name/Dockerfile ${HOME}
docker tag $image_plus_tag $image_name:latest
#docker tag $image_plus_tag $image_name:$hg_id

echo "Built $image_plus_tag and tagged as $image_name:latest" # and $image_name:$hg_id"
