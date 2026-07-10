// Minimal CDR decoding helpers for reading ROS2 (rosbag2) messages without a
// ROS2 installation. Message layouts are hand-vendored in the ros2_msgs
// headers; the CDR wire format is handled by the vendored Fast-CDR library.
#pragma once

#include <cstdint>
#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>

namespace ros2_msgs {

using Cdr = eprosima::fastcdr::Cdr;

// Every message serialized by rmw with the "cdr" format starts with a 4-byte
// XCDR1 encapsulation header ({0x00, 0x00|0x01 (BE|LE), options}), which
// read_encapsulation() consumes; it also sets the stream endianness and the
// alignment base for all subsequent reads.
template <typename Msg>
Msg decode(const uint8_t* buf, size_t size) {
  eprosima::fastcdr::FastBuffer fb(
      reinterpret_cast<char*>(const_cast<uint8_t*>(buf)), size);
  Cdr cdr(fb, Cdr::DEFAULT_ENDIAN, Cdr::DDS_CDR);
  cdr.read_encapsulation();
  Msg m;
  deserialize(cdr, m);
  return m;
}

}  // namespace ros2_msgs
