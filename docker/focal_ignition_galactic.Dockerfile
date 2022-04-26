# using Ubuntu 20.04 Focal Fossa image
FROM ubuntu:focal

# no prompts from apt-get
ARG DEBIAN_FRONTEND=noninteractive

# use bash as primary shell for RUN commands
SHELL [ "/bin/bash", "-c" ]

# "fast" source list for my area, can be redundant or negatively affect download speed
RUN apt-get update && apt-get -y install ca-certificates curl gnupg
COPY mirrors.list /etc/apt/sources.list

# installing initial setup packages
RUN apt-get update && apt-get -y install \
    git \
    curl \
    wget \
    build-essential \
    cmake \
    lsb-release \
    python3-pip \
    python3-dev \
    gnupg2 \
    libxext6 \
    libx11-6 \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    freeglut3-dev \
    net-tools \
    && apt-get -y autoremove \
    && apt-get clean