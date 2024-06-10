#!/usr/bin/env bash

function help {
    echo "Usage: $0 -P <prefix> [-p <platform>]"
    echo ""
    echo "-h                    show this help"
    echo "-p <platform>         specify platform (default=$platform)"
    echo "-P <prefix>           prebuild with prefix"
}

platform=linux/amd64
if [[ $(uname -i) = "aarch64" ]]; then
    platform=linux/arm64
fi
prefix=

while getopts "hp:P:" arg; do
    case $arg in
    h)
        help
        exit
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

# setup local docker registry for multiplatform support
docker compose up -d

# setup multiplatform builder
docker buildx create --use --name mybuilder --driver docker-container \
       --driver-opt network=host  # option to make the builder access to the registry on the localhost

# replace ros Dockerfile FROM instruction to replace base image
while read -r line; do
    sed 's=FROM.*=ARG\ FROM_IMAGE\
FROM\ $FROM_IMAGE=' "$line" > "$line.tmp"
done < <(find docker/docker_images/ -wholename */$ROS_DISTRO/*/Dockerfile)

# bake
if [[ -n $platform ]]; then
    docker buildx bake --set *.platform=$platform
else
    docker buildx bake
fi

docker pull localhost:5000/cabot-base-humble-desktop-custom-mesa
docker image tag localhost:5000/cabot-base-humble-desktop-custom-mesa ${prefix}__jammy-humble-custom-mesa
docker pull localhost:5000/cabot-base-humble-base-custom
docker image tag localhost:5000/cabot-base-humble-base-custom ${prefix}__jammy-humble-base-custom
