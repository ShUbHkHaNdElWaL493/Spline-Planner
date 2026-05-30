FROM ubuntu:22.04

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    software-properties-common \
    build-essential \
    cmake \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository ppa:sdurobotics/ur-rtde -y \
    && apt-get update \
    && apt-get install -y \
    libboost-system-dev \
    libboost-thread-dev \
    libeigen3-dev \
    libgsl-dev \
    librtde \
    librtde-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /spline_planner
COPY . .
RUN cmake -B build
RUN cmake --build build

EXPOSE 3000
CMD ["/bin/bash"]