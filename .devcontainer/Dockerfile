# See here for image contents: https://github.com/microsoft/vscode-dev-containers/tree/v0.238.0/containers/cpp/.devcontainer/base.Dockerfile

# [Choice] Debian / Ubuntu version (use Debian 11, Ubuntu 18.04/22.04 on local arm64/Apple Silicon): debian-11, debian-10, ubuntu-22.04, ubuntu-20.04, ubuntu-18.04

FROM nvidia/cuda:11.6.0-cudnn8-devel-ubuntu20.04

ARG REINSTALL_CMAKE_VERSION_FROM_SOURCE="none"

WORKDIR /workspace

RUN apt update && export DEBIAN_FRONTEND=noninteractive \
    && apt -y install --no-install-recommends sudo \
    git \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Optionally install the cmake for vcpkg
COPY ../reinstall-cmake.sh /tmp/
RUN if [ "${REINSTALL_CMAKE_VERSION_FROM_SOURCE}" != "none" ]; then \
        chmod +x /tmp/reinstall-cmake.sh && /tmp/reinstall-cmake.sh ${REINSTALL_CMAKE_VERSION_FROM_SOURCE}; \
    fi \
    && rm -f /tmp/reinstall-cmake.sh

RUN mkdir /usr/local/lib/pkgconfig

COPY ./conf/opencv4.pc /usr/local/lib/pkgconfig/

RUN git clone https://github.com/Itseez/opencv.git \
    && git clone https://github.com/Itseez/opencv_contrib.git \
    && cd opencv \
    && mkdir build \
    && cd build/ \
    && cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local .. \
    && make -j8 2>&1 |tee make.log \
    && sudo make install \
    && sudo ldconfig

