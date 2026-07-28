FROM osrf/ros:noetic-desktop-full

ARG KALIBR_REPOSITORY=https://github.com/ethz-asl/kalibr.git
ARG KALIBR_REF=master
ARG KALIBR_BUILD_JOBS=4

SHELL ["/bin/bash", "-c"]

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    autoconf \
    automake \
    doxygen \
    git \
    libblas-dev \
    libboost-all-dev \
    libeigen3-dev \
    liblapack-dev \
    libopencv-dev \
    libpoco-dev \
    libsuitesparse-dev \
    libtbb-dev \
    libv4l-dev \
    python3-catkin-tools \
    python3-dev \
    python3-igraph \
    python3-matplotlib \
    python3-osrf-pycommon \
    python3-pip \
    python3-pyx \
    python3-scipy \
    python3-tk \
    python3-wxgtk4.0 \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

ENV WORKSPACE=/catkin_ws

RUN mkdir -p "${WORKSPACE}/src" \
    && source /opt/ros/noetic/setup.bash \
    && cd "${WORKSPACE}" \
    && catkin init \
    && catkin config --extend /opt/ros/noetic \
    && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN git clone --recursive "${KALIBR_REPOSITORY}" "${WORKSPACE}/src/kalibr" \
    && git -C "${WORKSPACE}/src/kalibr" checkout "${KALIBR_REF}" \
    && git -C "${WORKSPACE}/src/kalibr" submodule update --init --recursive \
    && git -C "${WORKSPACE}/src/kalibr" rev-parse HEAD > /opt/kalibr_commit

RUN source /opt/ros/noetic/setup.bash \
    && cd "${WORKSPACE}" \
    && catkin build -j"${KALIBR_BUILD_JOBS}" -p"${KALIBR_BUILD_JOBS}"

COPY kalibr-entrypoint.sh /usr/local/bin/kalibr-entrypoint
COPY run_kalibr.py /usr/local/bin/run-kalibr
RUN chmod 0755 /usr/local/bin/kalibr-entrypoint /usr/local/bin/run-kalibr

ENV KALIBR_MANUAL_FOCAL_LENGTH_INIT=1
ENV MPLBACKEND=Agg
ENV MPLCONFIGDIR=/tmp/matplotlib
ENV ROS_HOME=/tmp/ros

ENTRYPOINT ["/usr/local/bin/kalibr-entrypoint"]
CMD ["run-kalibr", "/input", "/output"]
