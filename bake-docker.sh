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
export REGISTRY=registry:5000

# setup local docker registry for multiplatform support
if [[ -z $(docker ps -f "name=registry" -q) ]]; then
    docker network create registry-network
    docker run -d \
	--rm \
        --name registry \
        --network registry-network \
        -p 127.0.0.1:5000:5000 \
        registry:2.7
fi

# setup multiplatform builder
# docker buildx rm mybuilder
if [[ -z $(docker buildx ls | grep "mybuilder\*") ]]; then
    echo "mybuilder is not selected"
    if [[ -z $(docker buildx ls | grep mybuilder) ]]; then
	echo "creating mybuilde"
        docker buildx create --use --name mybuilder --driver docker-container \
           --config buildkitd.toml \
           --driver-opt network=registry-network  # option to make the builder access to the registry on the localhost
    else
	echo "use mybuilde"
	docker buildx use mybuilder
    fi
fi

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


docker buildx use default

docker pull localhost:5000/cabot-base-humble-desktop-custom-mesa
docker image tag localhost:5000/cabot-base-humble-desktop-custom-mesa ${prefix}__jammy-humble-custom-mesa
docker pull localhost:5000/cabot-base-humble-base-custom
docker image tag localhost:5000/cabot-base-humble-base-custom ${prefix}__jammy-humble-base-custom
