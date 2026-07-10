/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef DATASET_IO_ROSBAG2_H
#define DATASET_IO_ROSBAG2_H

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <set>

#include <basalt/io/dataset_io.h>
#include <basalt/utils/assert.h>
#include <basalt/utils/filesystem.h>

#include <sqlite3.h>
#include <mcap/reader.hpp>

#include <ros2_msgs/geometry_msgs.h>
#include <ros2_msgs/sensor_msgs.h>

namespace basalt {

// Reads ROS2 (rosbag2) bags without a ROS2 installation. Supports the
// sqlite3 (.db3, default up to Humble) and MCAP (.mcap, default since Iron)
// storage backends with CDR-serialized messages. Split bags (multiple
// storage files in one directory) are supported; compressed bags are not
// (run `ros2 bag decompress` first).

struct Ros2TopicInfo {
  std::string name;
  std::string type;
};

// Random-access handle for lazy image loading, the rosbag2 analog of
// rosbag::IndexEntry. key is the sqlite rowid or the mcap log time.
struct Ros2MessageRef {
  uint16_t file_idx;
  int64_t key;
};

class Ros2StorageReader {
 public:
  using MessageCb =
      std::function<void(const std::string& topic, int64_t recv_t_ns,
                         const uint8_t* data, size_t size, int64_t key)>;

  virtual ~Ros2StorageReader() = default;

  virtual std::vector<Ros2TopicInfo> topics() = 0;

  // Iterates all messages in receive-time order (full scan).
  virtual void forEachMessage(const MessageCb& cb) = 0;

  // Reads back the serialized bytes of a single message (lazy random access).
  virtual std::vector<uint8_t> readMessage(const std::string& topic,
                                           int64_t key) = 0;

  static std::unique_ptr<Ros2StorageReader> open(const std::string& path);
};

class SqliteStorageReader : public Ros2StorageReader {
 public:
  explicit SqliteStorageReader(const std::string& path) {
    if (sqlite3_open_v2(path.c_str(), &db, SQLITE_OPEN_READONLY, nullptr) !=
        SQLITE_OK) {
      std::cerr << "Could not open sqlite3 bag " << path << ": "
                << sqlite3_errmsg(db) << std::endl;
      std::abort();
    }

    if (sqlite3_prepare_v2(db, "SELECT data FROM messages WHERE id = ?1", -1,
                           &read_stmt, nullptr) != SQLITE_OK) {
      std::cerr << "Unexpected rosbag2 sqlite3 schema in " << path << ": "
                << sqlite3_errmsg(db) << std::endl;
      std::abort();
    }
  }

  ~SqliteStorageReader() override {
    sqlite3_finalize(read_stmt);
    sqlite3_close(db);
  }

  std::vector<Ros2TopicInfo> topics() override {
    // Selecting named columns is robust against columns added by newer
    // rosbag2 schema versions (offered_qos_profiles, type_description_hash).
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db, "SELECT name, type FROM topics", -1, &stmt,
                           nullptr) != SQLITE_OK) {
      std::cerr << "Unexpected rosbag2 sqlite3 schema: " << sqlite3_errmsg(db)
                << std::endl;
      std::abort();
    }

    std::vector<Ros2TopicInfo> res;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      Ros2TopicInfo info;
      info.name = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
      info.type = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
      res.push_back(info);
    }
    sqlite3_finalize(stmt);
    return res;
  }

  void forEachMessage(const MessageCb& cb) override {
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db,
                           "SELECT m.id, t.name, m.timestamp, m.data FROM "
                           "messages m JOIN topics t ON m.topic_id = t.id "
                           "ORDER BY m.timestamp",
                           -1, &stmt, nullptr) != SQLITE_OK) {
      std::cerr << "Unexpected rosbag2 sqlite3 schema: " << sqlite3_errmsg(db)
                << std::endl;
      std::abort();
    }

    while (sqlite3_step(stmt) == SQLITE_ROW) {
      int64_t id = sqlite3_column_int64(stmt, 0);
      std::string topic =
          reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
      int64_t timestamp = sqlite3_column_int64(stmt, 2);
      const uint8_t* data =
          static_cast<const uint8_t*>(sqlite3_column_blob(stmt, 3));
      size_t size = sqlite3_column_bytes(stmt, 3);

      checkNotCompressed(data, size);
      cb(topic, timestamp, data, size, id);
    }
    sqlite3_finalize(stmt);
  }

  std::vector<uint8_t> readMessage(const std::string& topic,
                                   int64_t key) override {
    UNUSED(topic);

    sqlite3_bind_int64(read_stmt, 1, key);
    if (sqlite3_step(read_stmt) != SQLITE_ROW) {
      std::cerr << "Could not read rosbag2 message with id " << key
                << std::endl;
      std::abort();
    }
    const uint8_t* data =
        static_cast<const uint8_t*>(sqlite3_column_blob(read_stmt, 0));
    std::vector<uint8_t> res(data, data + sqlite3_column_bytes(read_stmt, 0));
    sqlite3_reset(read_stmt);
    sqlite3_clear_bindings(read_stmt);
    return res;
  }

 private:
  static void checkNotCompressed(const uint8_t* data, size_t size) {
    static constexpr uint8_t ZSTD_MAGIC[] = {0x28, 0xB5, 0x2F, 0xFD};
    if (size >= 4 && std::memcmp(data, ZSTD_MAGIC, 4) == 0) {
      std::cerr << "Compressed rosbag2 is not supported. Decompress it first "
                   "with `ros2 bag decompress`."
                << std::endl;
      std::abort();
    }
  }

  sqlite3* db = nullptr;
  sqlite3_stmt* read_stmt = nullptr;
};

class McapStorageReader : public Ros2StorageReader {
 public:
  explicit McapStorageReader(const std::string& path) {
    auto status = reader.open(path);
    if (!status.ok()) {
      std::cerr << "Could not open mcap bag " << path << ": " << status.message
                << std::endl;
      std::abort();
    }

    // The summary section provides the chunk indexes needed for seeking.
    status = reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
    if (!status.ok()) {
      std::cerr << "Could not read mcap summary of " << path << ": "
                << status.message << std::endl;
      std::abort();
    }
  }

  std::vector<Ros2TopicInfo> topics() override {
    // channels() and schemas() return copies of the maps by value
    const auto channels = reader.channels();
    const auto schemas = reader.schemas();

    std::vector<Ros2TopicInfo> res;
    for (const auto& [id, channel] : channels) {
      UNUSED(id);
      auto it = schemas.find(channel->schemaId);
      if (it == schemas.end()) continue;
      res.push_back({channel->topic, it->second->name});
    }
    return res;
  }

  void forEachMessage(const MessageCb& cb) override {
    mcap::ReadMessageOptions options;
    options.readOrder = mcap::ReadMessageOptions::ReadOrder::LogTimeOrder;

    for (const mcap::MessageView& mv :
         reader.readMessages(problemCb, options)) {
      cb(mv.channel->topic, mv.message.logTime,
         reinterpret_cast<const uint8_t*>(mv.message.data), mv.message.dataSize,
         mv.message.logTime);
    }
  }

  std::vector<uint8_t> readMessage(const std::string& topic,
                                   int64_t key) override {
    // Indexed read of the chunk(s) containing log time == key; startTime is
    // inclusive, endTime exclusive.
    mcap::ReadMessageOptions options(key, key + 1);
    options.topicFilter = [&](std::string_view t) { return t == topic; };

    for (const mcap::MessageView& mv :
         reader.readMessages(problemCb, options)) {
      const uint8_t* data = reinterpret_cast<const uint8_t*>(mv.message.data);
      return std::vector<uint8_t>(data, data + mv.message.dataSize);
    }

    std::cerr << "Could not read mcap message on topic " << topic
              << " with log time " << key << std::endl;
    std::abort();
  }

 private:
  static void problemCb(const mcap::Status& status) {
    std::cerr << "mcap: " << status.message << std::endl;
  }

  mcap::McapReader reader;
};

inline std::unique_ptr<Ros2StorageReader> Ros2StorageReader::open(
    const std::string& path) {
  char magic[8] = {};
  std::ifstream f(path, std::ios::binary);
  f.read(magic, 8);

  if (std::memcmp(magic, "\x89MCAP0\r\n", 8) == 0)
    return std::unique_ptr<Ros2StorageReader>(new McapStorageReader(path));

  if (std::memcmp(magic, "SQLite f", 8) == 0)
    return std::unique_ptr<Ros2StorageReader>(new SqliteStorageReader(path));

  std::cerr << path << " is neither a sqlite3 nor an mcap rosbag2 file"
            << std::endl;
  std::abort();
}

class Rosbag2VioDataset : public VioDataset {
  std::vector<std::unique_ptr<Ros2StorageReader>> readers;
  std::mutex m;

  size_t num_cams;

  std::vector<int64_t> image_timestamps;
  std::vector<std::string> cam_topic_by_id;

  // vector of images for every timestamp
  // assumes vectors size is num_cams for every timestamp with null pointers
  // for missing frames
  std::unordered_map<int64_t, std::vector<std::optional<Ros2MessageRef>>>
      image_data_idx;

  Eigen::aligned_vector<AccelData> accel_data;
  Eigen::aligned_vector<GyroData> gyro_data;

  std::vector<int64_t> gt_timestamps;  // ordered gt timestamps
  Eigen::aligned_vector<Sophus::SE3d> gt_pose_data;

  int64_t mocap_to_imu_offset_ns = 0;

 public:
  ~Rosbag2VioDataset() {}

  size_t get_num_cams() const { return num_cams; }

  std::vector<int64_t>& get_image_timestamps() { return image_timestamps; }
  Eigen::aligned_vector<AccelData>& get_accel_data() { return accel_data; }
  Eigen::aligned_vector<GyroData>& get_gyro_data() { return gyro_data; }

  const std::vector<int64_t>& get_gt_timestamps() const {
    return gt_timestamps;
  }
  const Eigen::aligned_vector<Sophus::SE3d>& get_gt_pose_data() const {
    return gt_pose_data;
  }

  int64_t get_mocap_to_imu_offset_ns() const { return mocap_to_imu_offset_ns; }

  std::vector<ImageData> get_image_data(int64_t t_ns) {
    std::vector<ImageData> res(num_cams);

    auto it = image_data_idx.find(t_ns);

    if (it != image_data_idx.end())
      for (size_t i = 0; i < num_cams; i++) {
        ImageData& id = res[i];

        if (!it->second[i].has_value()) continue;

        const Ros2MessageRef& ref = *it->second[i];

        m.lock();
        std::vector<uint8_t> raw =
            readers[ref.file_idx]->readMessage(cam_topic_by_id[i], ref.key);
        m.unlock();

        ros2_msgs::Image img_msg =
            ros2_msgs::decode<ros2_msgs::Image>(raw.data(), raw.size());

        id.img.reset(new ManagedImage<uint16_t>(img_msg.width, img_msg.height));

        if (!img_msg.header.frame_id.empty() &&
            std::isdigit(img_msg.header.frame_id[0])) {
          id.exposure = std::stol(img_msg.header.frame_id) * 1e-9;
        } else {
          id.exposure = -1;
        }

        if (img_msg.encoding == "mono8") {
          uint16_t* data_out = id.img->ptr;
          for (const auto& data_in : img_msg.data) {
            *data_out = data_in << 8;
            ++data_out;
          }

        } else if (img_msg.encoding == "bgr8" || img_msg.encoding == "rgb8") {
          uint16_t* data_out = id.img->ptr;
          for (auto data_in = img_msg.data.begin();
               data_in < img_msg.data.end(); data_in += 3) {
            uint16_t gray =
                std::round((data_in[0] + data_in[1] + data_in[2]) / 3.0);
            *data_out = gray << 8;
            ++data_out;
          }
        } else if (img_msg.encoding == "mono16") {
          std::memcpy(id.img->ptr, img_msg.data.data(), img_msg.data.size());
        } else {
          std::cerr << "Encoding " << img_msg.encoding << " is not supported."
                    << std::endl;
          std::abort();
        }
      }

    return res;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  friend class Rosbag2IO;
};

class Rosbag2IO : public DatasetIoInterface {
 public:
  Rosbag2IO() {}

  void read(const std::string& path) {
    if (!fs::exists(path))
      std::cerr << "No dataset found in " << path << std::endl;

    data.reset(new Rosbag2VioDataset);

    for (const std::string& file : resolveStorageFiles(path))
      data->readers.push_back(Ros2StorageReader::open(file));

    // get topics; message types are matched like in the ROS1 reader, but by
    // ROS2 type names (accepting both pkg/msg/Type and pkg/Type spellings)
    std::set<std::string> cam_topics;
    std::string imu_topic;
    std::string mocap_topic;
    bool mocap_is_pose = false;
    std::string point_topic;

    for (const auto& reader : data->readers) {
      for (const Ros2TopicInfo& info : reader->topics()) {
        const std::string type = normalizeTypeName(info.type);

        if (type == "sensor_msgs/Image") {
          cam_topics.insert(info.name);
        } else if (type == "sensor_msgs/Imu" &&
                   info.name.rfind("/fcu", 0) != 0) {
          imu_topic = info.name;
        } else if (type == "geometry_msgs/TransformStamped" ||
                   type == "geometry_msgs/PoseStamped") {
          mocap_topic = info.name;
          mocap_is_pose = type == "geometry_msgs/PoseStamped";
        } else if (type == "geometry_msgs/PointStamped") {
          point_topic = info.name;
        }
      }
    }

    std::cout << "imu_topic: " << imu_topic << std::endl;
    std::cout << "mocap_topic: " << mocap_topic << std::endl;
    std::cout << "cam_topics: ";
    for (const std::string& s : cam_topics) std::cout << s << " ";
    std::cout << std::endl;

    std::map<std::string, int> topic_to_id;
    int idx = 0;
    for (const std::string& s : cam_topics) {
      topic_to_id[s] = idx;
      data->cam_topic_by_id.push_back(s);
      idx++;
    }

    data->num_cams = cam_topics.size();

    int num_msgs = 0;

    int64_t min_time = std::numeric_limits<int64_t>::max();
    int64_t max_time = std::numeric_limits<int64_t>::min();

    std::vector<ros2_msgs::TransformStamped> mocap_msgs;
    std::vector<ros2_msgs::PointStamped> point_msgs;

    std::vector<int64_t>
        system_to_imu_offset_vec;  // t_imu = t_system + system_to_imu_offset
    std::vector<int64_t> system_to_mocap_offset_vec;  // t_mocap = t_system +
                                                      // system_to_mocap_offset

    std::set<int64_t> image_timestamps;

    for (size_t file_idx = 0; file_idx < data->readers.size(); file_idx++) {
      data->readers[file_idx]->forEachMessage([&](const std::string& topic,
                                                  int64_t recv_t_ns,
                                                  const uint8_t* msg,
                                                  size_t size, int64_t key) {
        if (cam_topics.find(topic) != cam_topics.end()) {
          // decode only the header; pixel data is loaded lazily later
          ros2_msgs::Header header =
              ros2_msgs::decode<ros2_msgs::Header>(msg, size);
          int64_t timestamp_ns = header.stamp.toNSec();

          auto& img_vec = data->image_data_idx[timestamp_ns];
          if (img_vec.size() == 0) img_vec.resize(data->num_cams);

          img_vec[topic_to_id.at(topic)] =
              Ros2MessageRef{static_cast<uint16_t>(file_idx), key};
          image_timestamps.insert(timestamp_ns);

          min_time = std::min(min_time, timestamp_ns);
          max_time = std::max(max_time, timestamp_ns);
        }

        if (imu_topic == topic) {
          ros2_msgs::Imu imu_msg = ros2_msgs::decode<ros2_msgs::Imu>(msg, size);
          int64_t time = imu_msg.header.stamp.toNSec();

          data->accel_data.emplace_back();
          data->accel_data.back().timestamp_ns = time;
          data->accel_data.back().data = Eigen::Vector3d(
              imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y,
              imu_msg.linear_acceleration.z);

          data->gyro_data.emplace_back();
          data->gyro_data.back().timestamp_ns = time;
          data->gyro_data.back().data = Eigen::Vector3d(
              imu_msg.angular_velocity.x, imu_msg.angular_velocity.y,
              imu_msg.angular_velocity.z);

          min_time = std::min(min_time, time);
          max_time = std::max(max_time, time);

          system_to_imu_offset_vec.push_back(time - recv_t_ns);
        }

        if (mocap_topic == topic) {
          ros2_msgs::TransformStamped mocap_msg;

          if (mocap_is_pose) {
            ros2_msgs::PoseStamped mocap_pose_msg =
                ros2_msgs::decode<ros2_msgs::PoseStamped>(msg, size);

            mocap_msg.header = mocap_pose_msg.header;
            mocap_msg.transform.rotation = mocap_pose_msg.pose.orientation;
            mocap_msg.transform.translation.x = mocap_pose_msg.pose.position.x;
            mocap_msg.transform.translation.y = mocap_pose_msg.pose.position.y;
            mocap_msg.transform.translation.z = mocap_pose_msg.pose.position.z;
          } else {
            mocap_msg =
                ros2_msgs::decode<ros2_msgs::TransformStamped>(msg, size);
          }

          int64_t time = mocap_msg.header.stamp.toNSec();

          mocap_msgs.push_back(mocap_msg);

          system_to_mocap_offset_vec.push_back(time - recv_t_ns);
        }

        if (point_topic == topic) {
          ros2_msgs::PointStamped point_msg =
              ros2_msgs::decode<ros2_msgs::PointStamped>(msg, size);

          int64_t time = point_msg.header.stamp.toNSec();

          point_msgs.push_back(point_msg);

          system_to_mocap_offset_vec.push_back(time - recv_t_ns);
        }

        num_msgs++;
      });
    }

    data->image_timestamps.clear();
    data->image_timestamps.insert(data->image_timestamps.begin(),
                                  image_timestamps.begin(),
                                  image_timestamps.end());

    // split bags are scanned file by file, so re-establish global time
    // ordering (no-op for single-file bags)
    auto stamp_less = [](const auto& a, const auto& b) {
      return a.header.stamp.toNSec() < b.header.stamp.toNSec();
    };
    std::sort(mocap_msgs.begin(), mocap_msgs.end(), stamp_less);
    std::sort(point_msgs.begin(), point_msgs.end(), stamp_less);

    auto data_less = [](const auto& a, const auto& b) {
      return a.timestamp_ns < b.timestamp_ns;
    };
    std::sort(data->accel_data.begin(), data->accel_data.end(), data_less);
    std::sort(data->gyro_data.begin(), data->gyro_data.end(), data_less);

    if (system_to_mocap_offset_vec.size() > 0) {
      int64_t system_to_imu_offset = median(system_to_imu_offset_vec);

      int64_t system_to_mocap_offset = median(system_to_mocap_offset_vec);

      data->mocap_to_imu_offset_ns =
          system_to_imu_offset - system_to_mocap_offset;
    }

    data->gt_pose_data.clear();
    data->gt_timestamps.clear();

    if (!mocap_msgs.empty())
      for (size_t i = 0; i < mocap_msgs.size() - 1; i++) {
        const auto& mocap_msg = mocap_msgs[i];

        int64_t time = mocap_msg.header.stamp.toNSec();

        Eigen::Quaterniond q(
            mocap_msg.transform.rotation.w, mocap_msg.transform.rotation.x,
            mocap_msg.transform.rotation.y, mocap_msg.transform.rotation.z);

        Eigen::Vector3d t(mocap_msg.transform.translation.x,
                          mocap_msg.transform.translation.y,
                          mocap_msg.transform.translation.z);

        int64_t timestamp_ns = time + data->mocap_to_imu_offset_ns;
        data->gt_timestamps.emplace_back(timestamp_ns);
        data->gt_pose_data.emplace_back(q, t);
      }

    if (!point_msgs.empty())
      for (size_t i = 0; i < point_msgs.size() - 1; i++) {
        const auto& point_msg = point_msgs[i];

        int64_t time = point_msg.header.stamp.toNSec();

        Eigen::Vector3d t(point_msg.point.x, point_msg.point.y,
                          point_msg.point.z);

        int64_t timestamp_ns = time;  // + data->mocap_to_imu_offset_ns;
        data->gt_timestamps.emplace_back(timestamp_ns);
        data->gt_pose_data.emplace_back(Sophus::SO3d(), t);
      }

    std::cout << "Total number of messages: " << num_msgs << std::endl;
    std::cout << "Image size: " << data->image_data_idx.size() << std::endl;

    std::cout << "Min time: " << min_time << " max time: " << max_time
              << " mocap to imu offset: " << data->mocap_to_imu_offset_ns
              << std::endl;

    std::cout << "Number of mocap poses: " << data->gt_timestamps.size()
              << std::endl;
  }

  void reset() { data.reset(); }

  VioDatasetPtr get_data() { return data; }

 private:
  // "sensor_msgs/msg/Image" -> "sensor_msgs/Image"
  static std::string normalizeTypeName(const std::string& type) {
    std::string res = type;
    size_t pos = res.find("/msg/");
    if (pos != std::string::npos) res.erase(pos, 4);
    return res;
  }

  static int64_t median(std::vector<int64_t>& vec) {
    std::nth_element(vec.begin(), vec.begin() + vec.size() / 2, vec.end());
    return vec[vec.size() / 2];
  }

  static std::vector<std::string> resolveStorageFiles(const std::string& path) {
    if (!fs::is_directory(path)) return {path};

    std::vector<std::string> db3_files, mcap_files;

    for (const auto& entry : fs::directory_iterator(path)) {
      if (!fs::is_regular_file(entry.path())) continue;

      const std::string ext = entry.path().extension().string();
      if (ext == ".db3") {
        db3_files.push_back(entry.path().string());
      } else if (ext == ".mcap") {
        mcap_files.push_back(entry.path().string());
      } else if (ext == ".zstd") {
        std::cerr << "Found " << entry.path()
                  << ": file-compressed rosbag2 is not supported. Decompress "
                     "it first with `ros2 bag decompress`."
                  << std::endl;
        std::abort();
      }
    }

    checkMetadataNotCompressed(path);

    if (!db3_files.empty() && !mcap_files.empty()) {
      std::cerr << "Found both .db3 and .mcap storage files in " << path
                << ", this is not supported" << std::endl;
      std::abort();
    }

    std::vector<std::string>& files =
        db3_files.empty() ? mcap_files : db3_files;

    if (files.empty()) {
      std::cerr << "No rosbag2 storage files (.db3, .mcap) found in " << path
                << std::endl;
      std::abort();
    }

    // order split bags by their numeric suffix (bag_2 before bag_10); the
    // scan re-sorts by timestamp anyway, this only makes logs deterministic
    auto natural_key = [](const std::string& s) {
      std::string stem = fs::path(s).stem().string();
      size_t digits = 0;
      while (digits < stem.size() && std::isdigit(static_cast<unsigned char>(
                                         stem[stem.size() - 1 - digits]))) {
        digits++;
      }
      const long num =
          digits == 0 ? -1 : std::stol(stem.substr(stem.size() - digits));
      return std::make_tuple(stem.substr(0, stem.size() - digits), num, s);
    };
    std::sort(files.begin(), files.end(),
              [&](const std::string& a, const std::string& b) {
                return natural_key(a) < natural_key(b);
              });

    return files;
  }

  static void checkMetadataNotCompressed(const std::string& path) {
    std::ifstream f(fs::path(path) / "metadata.yaml");
    if (!f) return;

    std::string line;
    while (std::getline(f, line)) {
      size_t pos = line.find("compression_mode:");
      if (pos == std::string::npos) continue;

      std::string value = line.substr(pos + strlen("compression_mode:"));
      value.erase(
          std::remove_if(value.begin(), value.end(),
                         [](char c) {
                           return std::isspace(static_cast<unsigned char>(c)) ||
                                  c == '"' || c == '\'';
                         }),
          value.end());

      if (!value.empty() && value != "NONE") {
        std::cerr << "Compressed rosbag2 (compression_mode: " << value
                  << ") is not supported. Decompress it first with `ros2 bag "
                     "decompress`."
                  << std::endl;
        std::abort();
      }
    }
  }

  std::shared_ptr<Rosbag2VioDataset> data;
};

// Detects whether a path (--dataset-type bag) is a ROS2 bag. ROS1 bags are
// single files starting with "#ROSBAG V2.0"; ROS2 bags are directories
// containing metadata.yaml and/or .db3/.mcap storage files (a direct path to
// a storage file is also accepted).
inline bool isRos2Bag(const std::string& path) {
  if (fs::is_directory(path)) {
    if (fs::exists(fs::path(path) / "metadata.yaml")) return true;

    for (const auto& entry : fs::directory_iterator(path)) {
      const std::string ext = entry.path().extension().string();
      if (ext == ".db3" || ext == ".mcap") return true;
    }
    return false;
  }

  char magic[12] = {};
  std::ifstream f(path, std::ios::binary);
  f.read(magic, 12);

  if (std::memcmp(magic, "#ROSBAG V2.0", 12) == 0) return false;
  if (std::memcmp(magic, "\x89MCAP0\r\n", 8) == 0) return true;
  if (std::memcmp(magic, "SQLite f", 8) == 0) return true;

  return false;
}

}  // namespace basalt

#endif  // DATASET_IO_ROSBAG2_H
