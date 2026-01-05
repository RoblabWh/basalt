FROM docker.io/library/ubuntu:22.04

COPY . /basalt

RUN cd /basalt && \
    ## Install Dependencies
    DEBIAN_FRONTEND=noninteractive \
    ./scripts/install_ubuntu_deps.sh && \
    ## Build Software
    cmake -B build -G Ninja -D CMAKE_BUILD_TYPE=Release && \
    cmake --build build --parallel $(nproc) && \
    cmake --install build && \
    ldconfig

RUN useradd -m -s /bin/bash basalt
USER basalt
WORKDIR /home/basalt
