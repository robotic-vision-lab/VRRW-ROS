# using OSRF Galactic Desktop on Focal image
FROM osrf/ros:galactic-desktop-focal

# no prompts from apt-get
ARG DEBIAN_FRONTEND=noninteractive

# use bash as primary shell for RUN commands
SHELL [ "/bin/bash", "-c" ]

# YOU MAY WANT TO ADD --no-install-recommends TO REDUCE IMAGE SIZE

# for slow downloading mirror (no idea why this happens sometimes)
RUN apt-get update && apt-get -y install python3-pip apt-utils && \
    python3 -m pip install --upgrade pip setuptools wheel testresources && \
    python3 -m pip install --upgrade apt-mirror-updater && \
    apt-mirror-updater -a