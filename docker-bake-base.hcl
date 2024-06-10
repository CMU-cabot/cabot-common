group "default" {
  targets = ["cabot-base-amd64", "cabot-base-arm64"]
}

target "cabot-base-amd64" {
  dockerfile-inline = "FROM ubuntu:jammy"
  platforms = [ "linux/amd64" ]
  tags = [ "cmucal/cabot-base:amd64" ]
  output = [ "type=docker" ]
}

target "cabot-base-arm64" {
  dockerfile-inline = "FROM nvcr.io/nvidia/l4t-base:r36.2.0"
  platforms = [ "linux/arm64" ]
  tags = [ "cmucal/cabot-base:arm64" ]
  output = [ "type=docker" ]
}
