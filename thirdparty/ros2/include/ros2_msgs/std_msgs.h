// Hand-vendored ROS2 std_msgs message definitions with CDR deserialization.
// Field order and types must match the ROS2 .msg files.
#pragma once

#include <string>

#include <ros2_msgs/builtin_interfaces.h>
#include <ros2_msgs/cdr.h>

namespace ros2_msgs {

struct Header {
  Time stamp;
  std::string frame_id;  // note: no 'seq' field in ROS2
};

inline void deserialize(Cdr& c, Header& m) {
  deserialize(c, m.stamp);
  c >> m.frame_id;
}

}  // namespace ros2_msgs
