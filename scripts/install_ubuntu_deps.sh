#!/bin/bash
##
## BSD 3-Clause License
##
## This file is part of the Basalt project.
## https://gitlab.com/VladyslavUsenko/basalt.git
##
## Copyright (c) 2019-2021, Vladyslav Usenko and Nikolaus Demmel.
## All rights reserved.
##

deps=(
  # Build
  cmake ninja-build g++ git
  # Basalt
  libtbb-dev libeigen3-dev libopencv-dev libfmt-dev
  libboost-serialization-dev libboost-date-time-dev libboost-filesystem-dev libboost-program-options-dev libboost-regex-dev
  # Pangolin
  libglew-dev
  # Rosbag
  libbz2-dev liblz4-dev
  # Rosbag2
  libsqlite3-dev libzstd-dev
)

apt-get update
apt-get install --no-install-recommends -y "${deps[@]}"
