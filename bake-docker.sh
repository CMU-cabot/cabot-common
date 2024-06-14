#!/usr/bin/env bash

# Copyright (c) 2024  Carnegie Mellon University
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

# This script overview:
# - Builds the necessary images for cabot on multiple platforms and pushes them to cmucal.
#   - Since push requires cmucal permissions, it is usually handled by an Action script.
# - If you want to debug by modifying the base image, perform a local build (-l).
#   - Set up a local registry server, build, and overwrite the cmucal image tag.
#   - It is also advisable to specify the platform (-p) in this case.

function help {
    echo "Usage: $0 [-l] [-P <prefix>] [-p <platform>]"
    echo ""
    echo "-h                    show this help"
    echo "-d                    build only for the host machine platform"
    echo "-l                    build using local registry"
    echo "-p <platform>         specify platform"
    echo "                      build linux/arm64 and linux/amd64 if not specified"
    echo "-P <prefix>           prebuild with prefix (default=cabot-base)"
}

platform=
prefix=cabot
local=0

while getopts "hdlp:P:" arg; do
    case $arg in
    h)
        help
        exit
        ;;
    d)
	if [[ $(uname -m) = "x86_64" ]]; then
            platform="linux/amd64"
	elif [[ $(uname -m) = "aarch64" ]]; then
            platform="linux/arm64"
	fi
	;;
    l)
        local=1
        ;;
    p)
        platform=${OPTARG}
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

export ROS_DISTRO=humble
export UBUNTU_DISTRO=jammy

if [[ -z $(docker network ls | grep "registry-network") ]]; then
    docker network create registry-network
fi
if [[ $local -eq 1 ]]; then
    export REGISTRY=registry:5000
    # setup local docker registry for multiplatform support
    if [[ -z $(docker ps -f "name=registry" -q) ]]; then
        docker run -d \
        --rm \
            --name registry \
            --network registry-network \
            -p 127.0.0.1:5000:5000 \
            registry:2.7
    fi
fi

# setup multiplatform builder
# docker buildx rm mybuilder
if [[ -z $(docker buildx ls | grep "mybuilder\*") ]]; then
    echo "mybuilder is not selected"
    if [[ -z $(docker buildx ls | grep mybuilder) ]]; then
        echo "creating mybuilder"
        docker buildx create --use --name mybuilder --driver docker-container \
           --config buildkitd.toml \
           --driver-opt network=registry-network  # option to make the builder access to the registry on the localhost
    else
        echo "use mybuilder"
        docker buildx use mybuilder
    fi
fi

# replace ros Dockerfile FROM instruction to replace base image
# this isn't good, but cannot find alternative
if [[ -e ./docker/docker_images ]]; then
    while read -r line; do
        sed 's=FROM.*=ARG\ FROM_IMAGE\
    FROM\ $FROM_IMAGE=' "$line" > "$line.tmp"
    done < <(find docker/docker_images/ -wholename */$ROS_DISTRO/*/Dockerfile)
fi

# bake
if [[ -n $platform ]]; then
    com="docker buildx bake --set *.platform=\"$platform\" $@"
else
    com="docker buildx bake $@"
fi

echo $com
eval $com

# reset buildx builder to default
docker buildx use default

# copy images from local registry
# this can override image tag
if [[ $local -eq 1 ]]; then
    tags=($(eval "$com --print" 2> /dev/null | jq -r '.target[].tags[] | split("/")[-1]' | jq --raw-input | jq -r --slurp 'join(" ")'))
    for tag in "${tags[@]}"; do
        echo "Pulling tag ($tag) from $REGISTRY (platform=$(uname -m))"
        com="docker pull localhost:5000/$tag"
        echo $com
        eval $com
        com="docker image tag localhost:5000/$tag cmucal/$tag"
        echo $com
        eval $com
        com="docker image tag localhost:5000/$tag ${prefix}__$tag"
        echo $com
        eval $com
    done
fi
