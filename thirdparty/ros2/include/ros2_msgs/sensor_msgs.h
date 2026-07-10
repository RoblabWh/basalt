// Hand-vendored ROS2 sensor_msgs message definitions with CDR
// deserialization. Field order and types must match the ROS2 .msg files.
#pragma once

#include <array>
#include <string>
#include <vector>

#include <ros2_msgs/cdr.h>
#include <ros2_msgs/geometry_msgs.h>
#include <ros2_msgs/std_msgs.h>

namespace ros2_msgs {

struct Image {
  Header header;
  uint32_t height = 0;
  uint32_t width = 0;
  std::string encoding;
  uint8_t is_bigendian = 0;
  uint32_t step = 0;
  std::vector<uint8_t> data;
};

struct CompressedImage {
  Header header;
  std::string format;
  std::vector<uint8_t> data;
};

struct Imu {
  Header header;
  Quaternion orientation;
  std::array<double, 9> orientation_covariance = {};
  Vector3 angular_velocity;
  std::array<double, 9> angular_velocity_covariance = {};
  Vector3 linear_acceleration;
  std::array<double, 9> linear_acceleration_covariance = {};
};

inline void deserialize(Cdr& c, Image& m) {
  deserialize(c, m.header);
  c >> m.height >> m.width >> m.encoding >> m.is_bigendian >> m.step >> m.data;
}

inline void deserialize(Cdr& c, CompressedImage& m) {
  deserialize(c, m.header);
  c >> m.format >> m.data;
}

inline void deserialize(Cdr& c, Imu& m) {
  deserialize(c, m.header);
  deserialize(c, m.orientation);
  c >> m.orientation_covariance;
  deserialize(c, m.angular_velocity);
  c >> m.angular_velocity_covariance;
  deserialize(c, m.linear_acceleration);
  c >> m.linear_acceleration_covariance;
}

}  // namespace ros2_msgs
