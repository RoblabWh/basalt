FROM docker.io/library/ubuntu:24.04 AS base

# Install Dependencies
COPY scripts/install_ubuntu_deps.sh /tmp/install_ubuntu_deps.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache && \
    export DEBIAN_FRONTEND=noninteractive && \
    /tmp/install_ubuntu_deps.sh


FROM base AS devel

# Install development tools
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    export DEBIAN_FRONTEND=noninteractive && \
    apt-get update && \
    apt-get install --yes clangd clang-tidy clang-format


FROM base AS build

# Build Basalt
COPY . /basalt
RUN cd /basalt && \
    cmake -B build -G Ninja -D CMAKE_BUILD_TYPE=Release && \
    cmake --build build --parallel $(nproc) && \
    cmake --install build && \
    ldconfig


FROM build AS package

# Install deb packaging tools
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    export DEBIAN_FRONTEND=noninteractive && \
    apt-get update && \
    apt-get install --yes dpkg-dev file

# Package deb
RUN cmake --build /basalt/build --target package


FROM scratch AS deb

# Export deb: docker build --target deb --output type=local,dest=. .
COPY --from=package /basalt/build/*.deb /


FROM build AS runtime

# Configure runtime environment
USER ubuntu
WORKDIR /home/ubuntu
