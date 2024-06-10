#!/bin/bash

# build base image for amd64 and arm64 platform
# then make and push a multiplatform manifest 
docker buildx bake -f docker-bake-base.hcl
docker image push cmucal/cabot-base:amd64
docker image push cmucal/cabot-base:arm64
docker manifest create cmucal/cabot-base:latest \
  --amend cmucal/cabot-base:amd64 \
  --amend cmucal/cabot-base:arm64
docker manifest push cmucal/cabot-base:latest


#docker buildx create --use --name mybuilder --driver docker-container
#docker buildx inspect mybuilder --bootstrap
#docker buildx bake default
