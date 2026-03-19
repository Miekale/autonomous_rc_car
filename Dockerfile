FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV OPENCV_VERSION=4.9.0

# ------------------------
# System dependencies
# ------------------------
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libgtk-3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    gfortran \
    openexr \
    libatlas-base-dev \
    python3-dev \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# ------------------------
# Clone OpenCV + contrib
# ------------------------
WORKDIR /opt

RUN git clone https://github.com/opencv/opencv.git --branch ${OPENCV_VERSION} && \
    git clone https://github.com/opencv/opencv_contrib.git --branch ${OPENCV_VERSION}


# ------------------------
# Build OpenCV
# ------------------------
WORKDIR /opt/opencv
RUN mkdir build

WORKDIR /opt/opencv/build
RUN cmake \
    -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib/modules \
    -D WITH_TBB=ON \
    -D WITH_V4L=ON \
    -D WITH_OPENGL=ON \
    -D BUILD_EXAMPLES=OFF \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=OFF \
    ..

RUN make -j$(nproc)
RUN make install
RUN ldconfig

# ------------------------
# Default workdir for your C++ project
# ------------------------
WORKDIR /home/
