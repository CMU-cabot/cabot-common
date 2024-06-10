variable "PLATFORMS" {
  default = ["linux/arm64", "linux/amd64"]
}

variable "ROS_DISTRO" {
  default = "humble"
}

variable "UBUNTU_DISTRO" {
  default = "jammy"
}

variable "FROM_IMAGE" {
  default = "cmucal/cabot-base"
}

group "default" {
  targets = [
    "ros-core",
    "ros-base",
    "ros-desktop",
    "ros-base-custom",
    "ros-desktop-custom",
    "ros-desktop-custom-mesa"
  ]
}

target "ros-core" {
  context    = "./docker/docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/ros-core"
  dockerfile = "Dockerfile"
  platforms = "${PLATFORMS}"
  args      = { FROM_IMAGE = "${FROM_IMAGE}" }
  tags      = [ "${FROM_IMAGE}-${ROS_DISTRO}" ]
}

target "ros-base" {
  context    = "./docker/docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/ros-base"
  dockerfile = "Dockerfile"
  platforms = "${PLATFORMS}"
  args      = { FROM_IMAGE = "${FROM_IMAGE}-${ROS_DISTRO}" }
  tags      = [ "${FROM_IMAGE}-${ROS_DISTRO}-base" ]
  depends_on = ["ros-core"]
}

target "ros-desktop" {
  context    = "./docker/docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/desktop"
  dockerfile = "Dockerfile"
  platforms = "${PLATFORMS}"
  args      = { FROM_IMAGE = "${FROM_IMAGE}-${ROS_DISTRO}-base" }
  tags      = [ "${FROM_IMAGE}-${ROS_DISTRO}-desktop" ]
  depends_on = ["ros-base"]
}

target "ros-base-custom" {
  context    = "./docker/humble-custom"
  dockerfile = "Dockerfile"
  platforms = "${PLATFORMS}"
  args      = { FROM_IMAGE = "${FROM_IMAGE}-${ROS_DISTRO}-base" }
  tags      = [ "${FROM_IMAGE}-${ROS_DISTRO}-base-custom" ]
  output    = [ "type=docker" ]
  depends_on = ["ros-base"]
}

target "ros-desktop-custom" {
  context    = "./docker/humble-custom"
  dockerfile = "Dockerfile"
  platforms = "${PLATFORMS}"
  args      = { FROM_IMAGE = "${FROM_IMAGE}-${ROS_DISTRO}-desktop" }
  tags      = [ "${FROM_IMAGE}-${ROS_DISTRO}-desktop-custom" ]
  depends_on = ["ros-desktop"]
}

target "ros-desktop-custom-mesa" {
  context    = "./docker/mesa"
  dockerfile = "Dockerfile"
  platforms = "${PLATFORMS}"
  args      = { FROM_IMAGE = "${FROM_IMAGE}-${ROS_DISTRO}-desktop-custom" }
  tags      = [ "${FROM_IMAGE}-${ROS_DISTRO}-desktop-custom-mesa" ]
  depends_on = ["ros-desktop-custom"]
}



/*
target "ros-core" {
  name = "ros-core-${item.short}"
  context    = "./docker/docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/ros-core"
  dockerfile = "Dockerfile"
  matrix    = {
    item = [
      { short = "amd64", platform = "linux/amd64", from_image = "${FROM_IMAGE_AMD64}" },
      { short = "arm64", platform = "linux/arm64", from_image = "${FROM_IMAGE_ARM64}" },
    ]
  }
  platforms = [ item.platform ]
  args      = { FROM_IMAGE = item.from_image }
  tags      = [ "${item.from_image}-${ROS_DISTRO}" ]
}

target "ros-base" {
  name = "ros-base-${item.short}"
  context    = "./docker/docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/ros-base"
  dockerfile = "Dockerfile"
  matrix    = {
    item = [
      { short = "amd64", platform = "linux/amd64", from_image = "${FROM_IMAGE_AMD64}" },
      { short = "arm64", platform = "linux/arm64", from_image = "${FROM_IMAGE_ARM64}" },
    ]
  }
  platforms = [ item.platform ]
  args      = { FROM_IMAGE = "${item.from_image}-${ROS_DISTRO}" }
  tags      = [ "${item.from_image}-${ROS_DISTRO}-base" ]
}
*/

/*

target "ros-base" {
  context    = "./docker/docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/ros-base"
  dockerfile = "Dockerfile"
  args = {
    FROM_IMAGE = "${FROM_IMAGE}-${ROS_DISTRO}"
  }
  platforms  = [
    "linux/amd64",
    "linux/arm64"
  ]
  tags       = [
    "${FROM_IMAGE_AMD64}-${ROS_DISTRO}-base-amd64",
    "${FROM_IMAGE_ARM64}-${ROS_DISTRO}-base-arm64"
  ]
}

target "ros-desktop" {
  context    = "./docker/docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/desktop"
  dockerfile = "Dockerfile"
  args = {
    FROM_IMAGE = "${FROM_IMAGE}-${ROS_DISTRO}-base"
  }
  platforms  = [
    "linux/amd64",
    "linux/arm64"
  ]
  tags       = [
    "${FROM_IMAGE_AMD64}-${ROS_DISTRO}-desktop-amd64",
    "${FROM_IMAGE_ARM64}-${ROS_DISTRO}-desktop-arm64"
  ]
}

target "custom" {
  context    = "./docker/${ROS_DISTRO}-custom"
  dockerfile = "Dockerfile"
  args = {
    FROM_IMAGE = "${FROM_IMAGE}-${ROS_DISTRO}-desktop"
  }
  platforms  = [
    "linux/amd64",
    "linux/arm64"
  ]
  tags       = [
    "${FROM_IMAGE_AMD64}-${ROS_DISTRO}-desktop-amd64",
    "${FROM_IMAGE_ARM64}-${ROS_DISTRO}-desktop-arm64"
  ]
}
*/