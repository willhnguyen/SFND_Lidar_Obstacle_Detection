FROM nvidia/opengl:1.0-glvnd-devel-ubuntu18.04

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Los_Angeles
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update
RUN apt-get upgrade -y
RUN apt-get install -y \
    curl wget lsb-release build-essential sudo clang-format libpcl-dev cmake

WORKDIR /workspace
