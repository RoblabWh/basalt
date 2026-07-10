// Hand-vendored ROS2 builtin_interfaces message definitions with CDR
// deserialization. Field order and types must match the ROS2 .msg files.
#pragma once

#include <ros2_msgs/cdr.h>

namespace ros2_msgs {

struct Time {
  int32_t sec = 0;       // note: signed in ROS2 (was unsigned in ROS1)
  uint32_t nanosec = 0;

  int64_t toNSec() const {
    return static_cast<int64_t>(sec) * 1000000000 + nanosec;
  }
};

inline void deserialize(Cdr& c, Time& m) { c >> m.sec >> m.nanosec; }

}  // namespace ros2_msgs
