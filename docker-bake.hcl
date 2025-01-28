variable "PLATFORMS" {
  default = ["linux/arm64", "linux/amd64"]
}

variable "ROS_DISTRO" {
  default = "humble"
}

variable "UBUNTU_DISTRO" {
  default = "jammy"
}

variable "BASE_IMAGE" {
  default = "cabot-base"
}

variable "REGISTRY" {
  default = "cmucal"
}

group "default" {
  targets = [
    "base",
    "ros-core",
    "ros-base",
    "ros-desktop",
    "ros-base-custom",
    "ros-base-custom-mesa",
    "ros-desktop-custom",
    "ros-desktop-custom-mesa"
  ]
}

target "base" {
  dockerfile-inline = <<EOF
FROM --platform=linux/amd64 ubuntu:jammy as build-amd64
FROM --platform=linux/arm64 nvcr.io/nvidia/l4t-base:r36.2.0 as build-arm64
RUN sed -i.bak -r 's!http://ports.ubuntu.com/ubuntu-ports/!https://mirror.kumi.systems/ubuntu-ports/!' /etc/apt/sources.list && \
    cat /etc/apt/sources.list
FROM build-$TARGETARCH
EOF
  platforms  = "${PLATFORMS}"
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:base" ]
  output     = [ "type=registry" ]
}

target "ros-common" {
  platforms  = "${PLATFORMS}"
  output     = [ "type=registry" ]
}

target "ros-core" {
  inherits   = [ "ros-common" ]
  context    = "./docker/docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/ros-core"
  dockerfile = "Dockerfile.tmp"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:base" = "target:base" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:base" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-core" ]
}

target "ros-base" {
  inherits   = [ "ros-common" ]
  context    = "./docker/docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/ros-base"
  dockerfile = "Dockerfile.tmp"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-core" = "target:ros-core" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-core" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-base" ]
}

target "ros-desktop" {
  inherits   = [ "ros-common" ]
  context    = "./docker/docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/desktop"
  dockerfile = "Dockerfile.tmp"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-base" = "target:ros-base" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-base" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop" ]
}

target "ros-base-custom" {
  inherits   = [ "ros-common" ]
  context    = "./docker/humble-custom"
  dockerfile = "Dockerfile"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-base" = "target:ros-base" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-base" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-base-custom" ]
}

target "ros-base-custom-mesa" {
  inherits   = [ "ros-common" ]
  context    = "./docker/mesa"
  dockerfile = "Dockerfile"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-base-custom" = "target:ros-base-custom" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-base-custom" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-base-custom-mesa" ]
}

target "ros-desktop-custom" {
  inherits   = [ "ros-common" ]
  context    = "./docker/humble-custom"
  dockerfile = "Dockerfile"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop" = "target:ros-desktop" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom" ]
}

target "ros-desktop-custom-mesa" {
  inherits   = [ "ros-common" ]
  context    = "./docker/mesa"
  dockerfile = "Dockerfile"
  platforms  = "${PLATFORMS}"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom" = "target:ros-desktop-custom" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-mesa" ]
}
