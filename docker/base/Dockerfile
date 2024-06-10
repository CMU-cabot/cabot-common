FROM --platform=linux/amd64 ubuntu:jammy as build-amd64
FROM --platform=linux/arm64 nvcr.io/nvidia/l4t-base:r36.2.0 as build-arm64

FROM build-$TARGETARCH
