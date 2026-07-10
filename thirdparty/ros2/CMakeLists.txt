# Minimal vendored ROS2 rosbag2 reading support: Fast-CDR for CDR
# deserialization, the header-only mcap reader, and system sqlite3/zstd.
# No ROS2 installation is required, mirroring the approach of ../ros.
cmake_minimum_required(VERSION 3.14)  # FindSQLite3 needs >= 3.14

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wno-error -Wno-unused-parameter -Wno-deprecated-declarations")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")

find_package(SQLite3 REQUIRED)
find_library(lz4_LIBRARIES NAMES lz4 REQUIRED)
find_library(zstd_LIBRARIES NAMES zstd REQUIRED)
find_path(zstd_INCLUDE_DIR NAMES zstd.h REQUIRED)
find_path(lz4_INCLUDE_DIR NAMES lz4frame.h REQUIRED)

# Fast-CDR's option(BUILD_SHARED_LIBS ON) must not turn the build shared.
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Enable BUILD_SHARED_LIBS")
add_subdirectory(Fast-CDR EXCLUDE_FROM_ALL)

add_library(rosbag2 STATIC src/mcap_impl.cpp)

# Re-export fastcdr's include dirs (source + generated config.h) as SYSTEM so
# they enter the -Werror basalt target as -isystem.
get_target_property(FASTCDR_INCLUDE_DIRS fastcdr INTERFACE_INCLUDE_DIRECTORIES)

target_include_directories(rosbag2 SYSTEM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/mcap/cpp/mcap/include
    ${FASTCDR_INCLUDE_DIRS}
    ${SQLite3_INCLUDE_DIRS}
    ${zstd_INCLUDE_DIR}
    ${lz4_INCLUDE_DIR})

target_link_libraries(rosbag2 PUBLIC fastcdr ${SQLite3_LIBRARIES} ${zstd_LIBRARIES} ${lz4_LIBRARIES})
