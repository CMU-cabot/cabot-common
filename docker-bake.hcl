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
  default = "registry"
}

group "default" {
  targets = [
    "cabot-base",
    "ros-core",
    "ros-base",
    "ros-desktop",
    "ros-base-custom",
    # "ros-base-custom-mesa",
    "ros-desktop-custom",
    "ros-desktop-custom-mesa"
  ]
}

target "cabot-base" {
  dockerfile-inline = <<EOF
FROM --platform=linux/amd64 ubuntu:jammy as build-amd64
FROM --platform=linux/arm64 nvcr.io/nvidia/l4t-base:r36.2.0 as build-arm64
FROM build-$TARGETARCH
EOF
  platforms  = "${PLATFORMS}"
  tags       = [ "${REGISTRY}/cabot-base:latest" ]
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
  contexts   = { "${REGISTRY}/${BASE_IMAGE}" = "target:cabot-base" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}-${ROS_DISTRO}" ]
}

target "ros-base" {
  inherits   = [ "ros-common" ]
  context    = "./docker/docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/ros-base"
  dockerfile = "Dockerfile.tmp"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}-${ROS_DISTRO}" = "target:ros-core" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}-${ROS_DISTRO}" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}-${ROS_DISTRO}-base" ]
}

target "ros-desktop" {
  inherits   = [ "ros-common" ]
  context    = "./docker/docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/desktop"
  dockerfile = "Dockerfile.tmp"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}-${ROS_DISTRO}-base" = "target:ros-base" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}-${ROS_DISTRO}-base" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}-${ROS_DISTRO}-desktop" ]
}

target "ros-base-custom" {
  inherits   = [ "ros-common" ]
  context    = "./docker/humble-custom"
  dockerfile = "Dockerfile"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}-${ROS_DISTRO}-base" = "target:ros-base" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}-${ROS_DISTRO}-base" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}-${ROS_DISTRO}-base-custom" ]
}

target "ros-base-custom-mesa" {
  inherits   = [ "ros-common" ]
  context    = "./docker/mesa"
  dockerfile = "Dockerfile"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}-${ROS_DISTRO}-base" = "target:ros-base-custom" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}-${ROS_DISTRO}-base-custom" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}-${ROS_DISTRO}-base-custom-mesa" ]
}

target "ros-desktop-custom" {
  inherits   = [ "ros-common" ]
  context    = "./docker/humble-custom"
  dockerfile = "Dockerfile"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}-${ROS_DISTRO}-desktop" = "target:ros-desktop" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}-${ROS_DISTRO}-desktop" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}-${ROS_DISTRO}-desktop-custom" ]
}

target "ros-desktop-custom-mesa" {
  inherits   = [ "ros-common" ]
  context    = "./docker/mesa"
  dockerfile = "Dockerfile"
  platforms  = "${PLATFORMS}"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}-${ROS_DISTRO}-desktop-custom" = "target:ros-desktop-custom" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}-${ROS_DISTRO}-desktop-custom" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}-${ROS_DISTRO}-desktop-custom-mesa" ]
}
