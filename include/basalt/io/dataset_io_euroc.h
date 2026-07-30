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
#ifndef DATASET_IO_EUROC_H
#define DATASET_IO_EUROC_H

#include <map>
#include <set>

#include <basalt/io/dataset_io.h>
#include <basalt/utils/filesystem.h>

#include <opencv2/highgui/highgui.hpp>

namespace basalt {

class EurocVioDataset : public VioDataset {
  size_t num_cams;

  // names of the kept camera folders under mav0
  std::vector<std::string> cam_folders;

  std::string path;

  std::vector<int64_t> image_timestamps;
  std::unordered_map<int64_t, std::string> image_path;

  // vector of images for every timestamp
  // assumes vectors size is num_cams for every timestamp with null pointers for
  // missing frames
  // std::unordered_map<int64_t, std::vector<ImageData>> image_data;

  Eigen::aligned_vector<AccelData> accel_data;
  Eigen::aligned_vector<GyroData> gyro_data;

  std::vector<int64_t> gt_timestamps;  // ordered gt timestamps
  Eigen::aligned_vector<Sophus::SE3d>
      gt_pose_data;  // TODO: change to eigen aligned
  bool gt_position_only = false;

  int64_t mocap_to_imu_offset_ns = 0;

  std::vector<std::unordered_map<int64_t, double>> exposure_times;

 public:
  ~EurocVioDataset() {};

  size_t get_num_cams() const { return num_cams; }

  std::vector<std::string> get_cam_names() const { return cam_folders; }

  std::string get_imu_name() const { return "imu0"; }

  std::vector<int64_t> &get_image_timestamps() { return image_timestamps; }
  Eigen::aligned_vector<AccelData> &get_accel_data() { return accel_data; }
  Eigen::aligned_vector<GyroData> &get_gyro_data() { return gyro_data; }

  const std::vector<int64_t> &get_gt_timestamps() const {
    return gt_timestamps;
  }
  const Eigen::aligned_vector<Sophus::SE3d> &get_gt_pose_data() const {
    return gt_pose_data;
  }

  bool is_gt_position_only() const { return gt_position_only; }

  int64_t get_mocap_to_imu_offset_ns() const { return mocap_to_imu_offset_ns; }

  std::vector<ImageData> get_image_data(int64_t t_ns) {
    std::vector<ImageData> res(num_cams);

    for (size_t i = 0; i < num_cams; i++) {
      std::string full_image_path =
          path + "/mav0/" + cam_folders[i] + "/data/" + image_path[t_ns];

      if (fs::exists(full_image_path)) {
        cv::Mat img = cv::imread(full_image_path, cv::IMREAD_UNCHANGED);

        if (img.type() == CV_8UC1) {
          res[i].img.reset(new ManagedImage<uint16_t>(img.cols, img.rows));

          const uint8_t *data_in = img.ptr();
          uint16_t *data_out = res[i].img->ptr;

          size_t full_size = img.cols * img.rows;
          for (size_t i = 0; i < full_size; i++) {
            int val = data_in[i];
            val = val << 8;
            data_out[i] = val;
          }
        } else if (img.type() == CV_8UC3) {
          res[i].img.reset(new ManagedImage<uint16_t>(img.cols, img.rows));

          const uint8_t *data_in = img.ptr();
          uint16_t *data_out = res[i].img->ptr;

          size_t full_size = img.cols * img.rows;
          for (size_t i = 0; i < full_size; i++) {
            int val = data_in[i * 3];
            val = val << 8;
            data_out[i] = val;
          }
        } else if (img.type() == CV_16UC1) {
          res[i].img.reset(new ManagedImage<uint16_t>(img.cols, img.rows));
          std::memcpy(res[i].img->ptr, img.ptr(),
                      img.cols * img.rows * sizeof(uint16_t));

        } else {
          std::cerr << "img.fmt.bpp " << img.type() << std::endl;
          std::abort();
        }

        auto exp_it = exposure_times[i].find(t_ns);
        if (exp_it != exposure_times[i].end()) {
          res[i].exposure = exp_it->second;
        }
      }
    }

    return res;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  friend class EurocIO;
};

class EurocIO : public DatasetIoInterface {
 public:
  using DatasetIoInterface::DatasetIoInterface;

  void read(const std::string &path) {
    if (!fs::exists(path))
      std::cerr << "No dataset found in " << path << std::endl;

    data.reset(new EurocVioDataset);

    // ground-truth sources present on disk
    enum class GtFormat { state, pose, position };
    std::map<std::string, GtFormat> gt_sources;
    for (const auto &[name, format] :
         {std::pair<std::string, GtFormat>{"state_groundtruth_estimate0",
                                           GtFormat::state},
          {"gt", GtFormat::pose},
          {"mocap0", GtFormat::pose},
          {"leica0", GtFormat::position}}) {
      if (fs::exists(path + "/mav0/" + name + "/data.csv"))
        gt_sources[name] = format;
    }

    std::vector<std::string> all_sensor_names = {"cam0", "cam1", "imu0"};
    for (const auto &kv : gt_sources) all_sensor_names.push_back(kv.first);

    for (const std::string cam : {"cam0", "cam1"}) {
      if (sensor_filter.keep(cam, SensorTypes::Camera))
        data->cam_folders.push_back(cam);
    }
    sensor_filter.warn_unmatched(all_sensor_names);

    std::set<std::string> gt_selected;
    for (const auto &kv : gt_sources)
      if (sensor_filter.keep(kv.first, SensorTypes::GroundTruth))
        gt_selected.insert(kv.first);
    abort_if_multiple_topics("ground-truth", gt_selected);

    data->num_cams = data->cam_folders.size();
    data->path = path;

    if (!data->cam_folders.empty())
      read_image_timestamps(path + "/mav0/" + data->cam_folders[0] + "/");

    if (sensor_filter.keep("imu0", SensorTypes::Imu))
      read_imu_data(path + "/mav0/imu0/");

    if (!gt_selected.empty()) {
      const std::string &name = *gt_selected.begin();
      const std::string gt_path = path + "/mav0/" + name + "/";
      switch (gt_sources.at(name)) {
        case GtFormat::state:
          read_gt_data_state(gt_path);
          break;
        case GtFormat::pose:
          read_gt_data_pose(gt_path);
          break;
        case GtFormat::position:
          read_gt_data_position(gt_path);
          data->gt_position_only = true;
          break;
      }
    }

    data->exposure_times.resize(data->num_cams);
    for (size_t i = 0; i < data->cam_folders.size(); i++) {
      const std::string cam_path = path + "/mav0/" + data->cam_folders[i] + "/";
      if (fs::exists(cam_path + "exposure.csv")) {
        std::cout << "Loading exposure times for " << data->cam_folders[i]
                  << std::endl;
        read_exposure(cam_path, data->exposure_times[i]);
      }
    }
  }

  void reset() { data.reset(); }

  VioDatasetPtr get_data() { return data; }

 private:
  void read_exposure(const std::string &path,
                     std::unordered_map<int64_t, double> &exposure_data) {
    exposure_data.clear();

    std::ifstream f(path + "exposure.csv");
    std::string line;
    while (std::getline(f, line)) {
      if (line[0] == '#') continue;

      std::stringstream ss(line);

      char tmp;
      int64_t timestamp, exposure_int;
      Eigen::Vector3d gyro, accel;

      ss >> timestamp >> tmp >> exposure_int;

      exposure_data[timestamp] = exposure_int * 1e-9;
    }
  }

  void read_image_timestamps(const std::string &path) {
    std::ifstream f(path + "data.csv");
    std::string line;
    while (std::getline(f, line)) {
      if (line[0] == '#') continue;
      std::stringstream ss(line);
      char tmp;
      int64_t t_ns;
      std::string path;
      ss >> t_ns >> tmp >> path;

      data->image_timestamps.emplace_back(t_ns);
      data->image_path[t_ns] = path;
    }
  }

  void read_imu_data(const std::string &path) {
    data->accel_data.clear();
    data->gyro_data.clear();

    std::ifstream f(path + "data.csv");
    std::string line;
    while (std::getline(f, line)) {
      if (line[0] == '#') continue;

      std::stringstream ss(line);

      char tmp;
      uint64_t timestamp;
      Eigen::Vector3d gyro, accel;

      ss >> timestamp >> tmp >> gyro[0] >> tmp >> gyro[1] >> tmp >> gyro[2] >>
          tmp >> accel[0] >> tmp >> accel[1] >> tmp >> accel[2];

      data->accel_data.emplace_back();
      data->accel_data.back().timestamp_ns = timestamp;
      data->accel_data.back().data = accel;

      data->gyro_data.emplace_back();
      data->gyro_data.back().timestamp_ns = timestamp;
      data->gyro_data.back().data = gyro;
    }
  }

  void read_gt_data_state(const std::string &path) {
    data->gt_timestamps.clear();
    data->gt_pose_data.clear();

    std::ifstream f(path + "data.csv");
    std::string line;
    while (std::getline(f, line)) {
      if (line[0] == '#') continue;

      std::stringstream ss(line);

      char tmp;
      uint64_t timestamp;
      Eigen::Quaterniond q;
      Eigen::Vector3d pos, vel, accel_bias, gyro_bias;

      ss >> timestamp >> tmp >> pos[0] >> tmp >> pos[1] >> tmp >> pos[2] >>
          tmp >> q.w() >> tmp >> q.x() >> tmp >> q.y() >> tmp >> q.z() >> tmp >>
          vel[0] >> tmp >> vel[1] >> tmp >> vel[2] >> tmp >> accel_bias[0] >>
          tmp >> accel_bias[1] >> tmp >> accel_bias[2] >> tmp >> gyro_bias[0] >>
          tmp >> gyro_bias[1] >> tmp >> gyro_bias[2];

      data->gt_timestamps.emplace_back(timestamp);
      data->gt_pose_data.emplace_back(q, pos);
    }
  }

  void read_gt_data_pose(const std::string &path) {
    data->gt_timestamps.clear();
    data->gt_pose_data.clear();

    std::ifstream f(path + "data.csv");
    std::string line;
    while (std::getline(f, line)) {
      if (line[0] == '#') continue;

      std::stringstream ss(line);

      char tmp;
      uint64_t timestamp;
      Eigen::Quaterniond q;
      Eigen::Vector3d pos;

      ss >> timestamp >> tmp >> pos[0] >> tmp >> pos[1] >> tmp >> pos[2] >>
          tmp >> q.w() >> tmp >> q.x() >> tmp >> q.y() >> tmp >> q.z();

      data->gt_timestamps.emplace_back(timestamp);
      data->gt_pose_data.emplace_back(q, pos);
    }
  }

  void read_gt_data_position(const std::string &path) {
    data->gt_timestamps.clear();
    data->gt_pose_data.clear();

    std::ifstream f(path + "data.csv");
    std::string line;
    while (std::getline(f, line)) {
      if (line[0] == '#') continue;

      std::stringstream ss(line);

      char tmp;
      uint64_t timestamp;
      Eigen::Vector3d pos;

      ss >> timestamp >> tmp >> pos[0] >> tmp >> pos[1] >> tmp >> pos[2];

      data->gt_timestamps.emplace_back(timestamp);
      data->gt_pose_data.emplace_back(Eigen::Quaterniond::Identity(), pos);
    }
  }

  std::shared_ptr<EurocVioDataset> data;
};  // namespace basalt

}  // namespace basalt

#endif  // DATASET_IO_H
