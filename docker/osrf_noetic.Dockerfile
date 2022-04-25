# using OSRF Noetic Desktop on Focal image
FROM osrf/ros:noetic-desktop-full-focal

# no prompts from apt-get
ARG DEBIAN_FRONTEND=noninteractive

# use bash as primary shell for RUN commands
SHELL [ "/bin/bash", "-c" ]

# "fast" source list for my area, can be redundant or negatively affect download speed
RUN apt-get update && apt-get -y install ca-certificates curl gnupg
COPY mirrors.list /etc/apt/sources.list