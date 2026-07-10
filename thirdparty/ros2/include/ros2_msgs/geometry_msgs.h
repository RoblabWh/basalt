// Hand-vendored ROS2 geometry_msgs message definitions with CDR
// deserialization. Field order and types must match the ROS2 .msg files.
#pragma once

#include <string>

#include <ros2_msgs/cdr.h>
#include <ros2_msgs/std_msgs.h>

namespace ros2_msgs {

struct Vector3 {
  double x = 0, y = 0, z = 0;
};

struct Point {
  double x = 0, y = 0, z = 0;
};

struct Quaternion {
  double x = 0, y = 0, z = 0, w = 1;
};

struct Transform {
  Vector3 translation;
  Quaternion rotation;
};

struct TransformStamped {
  Header header;
  std::string child_frame_id;
  Transform transform;
};

struct Pose {
  Point position;
  Quaternion orientation;
};

struct PoseStamped {
  Header header;
  Pose pose;
};

struct PointStamped {
  Header header;
  Point point;
};

inline void deserialize(Cdr& c, Vector3& m) { c >> m.x >> m.y >> m.z; }

inline void deserialize(Cdr& c, Point& m) { c >> m.x >> m.y >> m.z; }

inline void deserialize(Cdr& c, Quaternion& m) {
  c >> m.x >> m.y >> m.z >> m.w;
}

inline void deserialize(Cdr& c, Transform& m) {
  deserialize(c, m.translation);
  deserialize(c, m.rotation);
}

inline void deserialize(Cdr& c, TransformStamped& m) {
  deserialize(c, m.header);
  c >> m.child_frame_id;
  deserialize(c, m.transform);
}

inline void deserialize(Cdr& c, Pose& m) {
  deserialize(c, m.position);
  deserialize(c, m.orientation);
}

inline void deserialize(Cdr& c, PoseStamped& m) {
  deserialize(c, m.header);
  deserialize(c, m.pose);
}

inline void deserialize(Cdr& c, PointStamped& m) {
  deserialize(c, m.header);
  deserialize(c, m.point);
}

}  // namespace ros2_msgs
