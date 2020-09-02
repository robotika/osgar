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
while getopts h arg; do
    case $arg in
        h)
            echo "Usage: $0 <directory-name>"
            echo "Requires the following folders to be present in $HOME/space-challenge/"
            echo "* srcp2-competitors from https://gitlab.com/scheducation/srcp2-competitors/"
            echo "* osgar .. this repository"
            exit 1
            ;;
    esac
done

DIR_ARG=${@:$OPTIND:1}

# get path to current directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

if [ ! -d $DIR/$DIR_ARG ]
then
  echo "image-name must be a directory in the same folder as this script"
  exit 2
fi

if [[ ! -d $HOME/space-challenge/srcp2-competitors || ! -d $HOME/space-challenge/osgar  ]]; then
    echo "Required folders missing from staging directory, run with -h switch for help"
    exit 2
fi

user_id=$(id -u)
image_name=$(basename $DIR_ARG)
image_plus_tag=$image_name:$(date +%Y_%m_%d_%H%M)
# mercurial adds a + symbol if there are uncomitted changes in the repo
# that will break docker tag syntax
#hg_id=$(hg id -i | tr -d '+')

shift


docker build --rm -t $image_plus_tag -f $DIR/$image_name/Dockerfile ${HOME}/space-challenge/ --build-arg NUM_THREADS=$(nproc)
docker tag $image_plus_tag $image_name:latest
#docker tag $image_plus_tag $image_name:$hg_id

echo "Built $image_plus_tag and tagged as $image_name:latest" # and $image_name:$hg_id"
