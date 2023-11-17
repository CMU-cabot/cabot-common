#!/bin/bash

# Copyright (c) 2020, 2023  Carnegie Mellon University, IBM Corporation, and others
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

## termination hook
trap ctrl_c INT QUIT TERM

function ctrl_c() {
    exit
}

function red {
    echo -en "\033[31m"  ## red
    echo $1
    echo -en "\033[0m"  ## reset color
}
function blue {
    echo -en "\033[36m"  ## blue
    echo $1
    echo -en "\033[0m"  ## reset color
}
function help {
    echo "Usage: $0 -p <prefix>"
    echo ""
    echo "-h                    show this help"
    echo "-P <prefix>           prebuild with prefix"
}

ROS2_UBUNTUV=22.04
ROS2_UBUNTU_DISTRO=jammy
ROS2_DISTRO=humble

pwd=$(pwd)
scriptdir=$(dirname $0)
cd $scriptdir
scriptdir=$(pwd)
build_dir=$scriptdir/docker

prefix=
option="--progress=auto"

while getopts "hP:" arg; do
    case $arg in
    h)
        help
        exit
        ;;
    P)
        prefix=${OPTARG}
        ;;
    esac
done
shift $((OPTIND-1))

if [[ -z $prefix ]]; then
    help
    exit 1
fi


function build_ros_base_image {
    local FROM_IMAGE=$1
    local IMAGE_TAG_PREFIX=$2
    local UBUNTU_DISTRO=$3
    local ROS_DISTRO=$4
    local ROS_COMPONENT=$5
    local -n IMAGE_TAG=$6

    echo ""
    IMAGE_TAG=$IMAGE_TAG_PREFIX-$ROS_DISTRO
    blue "## build $IMAGE_TAG"
    pushd $build_dir/docker_images/ros/$ROS_DISTRO/ubuntu/$UBUNTU_DISTRO/ros-core/

    sed s=FROM.*=FROM\ $FROM_IMAGE= Dockerfile > Dockerfile.temp && \
        docker build -f Dockerfile.temp -t $IMAGE_TAG $option .
    if [ $? -ne 0 ]; then
        red "failed to build $IMAGE_TAG"
        exit 1
    fi
    popd

    echo ""
    FROM_IMAGE=$IMAGE_TAG
    IMAGE_TAG=$IMAGE_TAG_PREFIX-$ROS_DISTRO-base
    blue "## build $IMAGE_TAG"
    pushd $build_dir/docker_images/ros/$ROS_DISTRO/ubuntu/$UBUNTU_DISTRO/ros-base/
    sed s=FROM.*=FROM\ $FROM_IMAGE= Dockerfile > Dockerfile.temp && \
        docker build -f Dockerfile.temp -t $IMAGE_TAG $option .
    if [ $? -ne 0 ]; then
        red "failed to build $IMAGE_TAG"
        exit 1
    fi
    popd

    if [[ $ROS_COMPONENT = "ros-base" ]]; then
    returnn
    fi

    echo ""
    IMAGE_TAG=$IMAGE_TAG_PREFIX-$ROS_DISTRO-desktop
    blue "## build $IMAGE_TAG"
    pushd $build_dir/docker_images/ros/$ROS_DISTRO/ubuntu/$UBUNTU_DISTRO/desktop/
    sed s=FROM.*=FROM\ $FROM_IMAGE= Dockerfile > Dockerfile.temp && \
        docker build -f Dockerfile.temp -t $IMAGE_TAG $option .
    if [ $? -ne 0 ]; then
        red "failed to build $IMAGE_TAG"
        exit 1
    fi
    popd
}

function prebuild {
    local FROM_IMAGE=$1
    local IMAGE_BASE_NAME=$2
    local IMAGE_DIR=$3
    local -n IMAGE_TAG=$4         # output variable name

    IMAGE_TAG=$IMAGE_BASE_NAME-$(basename $IMAGE_DIR)

    pushd $IMAGE_DIR
    blue "## build $IMAGE_TAG"
    docker build -t $IMAGE_TAG \
       --file Dockerfile \
       --build-arg TZ=$time_zone \
       --build-arg FROM_IMAGE=$FROM_IMAGE \
       . && popd
}

function prebuild_ros2 {
    blue "- UBUNTU_DISTRO=$ROS2_UBUNTU_DISTRO"
    blue "- ROS2_DISTRO=$ROS2_DISTRO"
    blue "- TIME_ZONE=$time_zone"

    base_image=ubuntu:$ROS2_UBUNTU_DISTRO
    base_name=${prefix}__${ROS2_UBUNTU_DISTRO}
    image_tag=$base_image
    build_ros_base_image $image_tag $image_tag $ROS2_UBUNTU_DISTRO $ROS2_DISTRO desktop image_tag
    if [ $? -ne 0 ]; then
    red "failed to build $name1"
    exit 1
    fi

    prebuild $image_tag $base_name $build_dir/${ROS2_DISTRO}-custom image_tag
    if [ $? -ne 0 ]; then
    red "failed to build $image_tag"
    return 1
    fi

    prebuild $image_tag $image_tag $build_dir/mesa image_tag
    if [ $? -ne 0 ]; then
    red "failed to build $image_tag"
    return 1
    fi
}


prebuild_ros2
