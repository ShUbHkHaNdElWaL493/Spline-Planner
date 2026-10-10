FROM ubuntu:24.04 AS spline-planner-build

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    software-properties-common \
    build-essential \
    clang-format \
    cmake \
    gnuplot \
    gfortran \
    libboost-system-dev \
    libboost-thread-dev \
    libeigen3-dev \
    libgsl-dev \
    && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository ppa:sdurobotics/ur-rtde -y \
    && apt-get update && apt-get install -y \
    librtde \
    librtde-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /spline_planner
COPY . .
RUN cmake -B build-debug -DCMAKE_BUILD_TYPE=Debug \
    && cmake -B build-release -DCMAKE_BUILD_TYPE=Release

RUN cmake --build build-debug \
    && cmake --build build-release

FROM spline-planner-build AS spline-planner-dev

EXPOSE 3000
CMD ["bash"]

FROM ubuntu:24.04 AS spline-planner-prod

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    software-properties-common \
    gnuplot \
    libboost-system-dev \
    libboost-thread-dev \
    libeigen3-dev \
    libgsl27 \
    && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository ppa:sdurobotics/ur-rtde -y \
    && apt-get update && apt-get install -y \
    librtde \
    librtde-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /spline_planner

COPY resources resources

COPY --from=spline-planner-build /spline_planner/spline_planner /spline_planner/spline_planner

CMD ["bash"]