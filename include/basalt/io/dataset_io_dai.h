#ifndef DATASET_IO_DAI_H
#define DATASET_IO_DAI_H

#include <basalt/io/dataset_io.h>
#include <basalt/utils/filesystem.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace basalt {

struct CameraMetadata {
  std::string name;
  fs::path basepath;
  std::vector<int64_t> timestamps;  // ns
  std::unordered_map<int64_t, fs::path> filenames;
  std::unordered_map<int64_t, double> exposures;  // ms
};

class DaiVioDataset : public VioDataset {
  std::vector<CameraMetadata> cam_meta;
  std::vector<int64_t> image_timestamps;

  Eigen::aligned_vector<AccelData> accel_data;
  Eigen::aligned_vector<GyroData> gyro_data;

  std::vector<int64_t> gt_timestamps;  // ordered gt timestamps
  Eigen::aligned_vector<Sophus::SE3d>
      gt_pose_data;  // TODO: change to eigen aligned

  int64_t mocap_to_imu_offset_ns = 0;

  // use ffmpeg to read lossless jpeg
  cv::Mat read_image(fs::path path) {
    cv::Mat img;
    cv::VideoCapture cap(path, cv::CAP_FFMPEG);
    if (cap.isOpened()) cap.read(img);
    return img;
  }

 public:
  ~DaiVioDataset() {};

  size_t get_num_cams() const { return cam_meta.size(); }

  std::vector<std::string> get_cam_names() const {
    std::vector<std::string> names;
    names.reserve(cam_meta.size());
    for (const CameraMetadata &cam : cam_meta) names.push_back(cam.name);
    return names;
  }

  std::vector<int64_t> &get_image_timestamps() { return image_timestamps; }
  Eigen::aligned_vector<AccelData> &get_accel_data() { return accel_data; }
  Eigen::aligned_vector<GyroData> &get_gyro_data() { return gyro_data; }

  const std::vector<int64_t> &get_gt_timestamps() const {
    return gt_timestamps;
  }
  const Eigen::aligned_vector<Sophus::SE3d> &get_gt_pose_data() const {
    return gt_pose_data;
  }

  int64_t get_mocap_to_imu_offset_ns() const { return mocap_to_imu_offset_ns; }

  std::vector<ImageData> get_image_data(int64_t timestamp) {
    std::vector<ImageData> res;
    res.reserve(cam_meta.size());

    for (const auto &cam : cam_meta) {
      auto &resimg = res.emplace_back();
      if (cam.filenames.count(timestamp)) {
        const auto &fullpath = cam.basepath / cam.filenames.at(timestamp);

        if (fs::exists(fullpath)) {
          cv::Mat img = read_image(fullpath);
          if (img.empty()) {
            std::cerr << "Failed to read image \"" << fullpath << "\" skipping"
                      << std::endl;
            continue;
          }

          if (img.type() == CV_8UC3 || img.type() == CV_16UC3) {
            cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
          } else if (img.type() == CV_8UC4 || img.type() == CV_16UC4) {
            cv::cvtColor(img, img, cv::COLOR_BGRA2GRAY);
          }

          if (img.type() == CV_8UC1) {
            resimg.img.reset(new ManagedImage<uint16_t>(img.cols, img.rows));

            const uint8_t *data_in = img.ptr();
            uint16_t *data_out = resimg.img->ptr;

            size_t full_size = img.cols * img.rows;
            for (size_t i = 0; i < full_size; i++) {
              int val = data_in[i];
              val = val << 8;
              data_out[i] = val;
            }
          } else if (img.type() == CV_16UC1) {
            resimg.img.reset(new ManagedImage<uint16_t>(img.cols, img.rows));
            std::memcpy(resimg.img->ptr, img.ptr(),
                        img.cols * img.rows * sizeof(uint16_t));

          } else {
            std::cerr << "img.fmt.bpp " << img.type() << std::endl;
            std::abort();
          }

          resimg.exposure = cam.exposures.at(timestamp);
        }
      }
    }

    return res;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  friend class DaiIO;
};

class DaiIO : public DatasetIoInterface {
 public:
  using DatasetIoInterface::DatasetIoInterface;

  void read(const std::string &path) {
    data.reset(new DaiVioDataset);

    fs::path root = path;
    if (!fs::is_directory(root)) {
      std::cerr << "No dataset found in " << path << std::endl;
      return;
    }

    std::vector<std::string> all_sensor_names;
    std::vector<fs::path> metadata_paths;
    const auto cams_path = root / "cams";
    if (fs::is_directory(cams_path)) {
      for (const auto &entry : fs::directory_iterator(cams_path)) {
        const auto cam_path = cams_path / entry.path();
        if (cam_path.extension() == ".csv" &&
            fs::is_directory(cams_path / cam_path.stem())) {
          all_sensor_names.push_back(cam_path.stem().string());
          if (sensor_filter.keep(cam_path.stem().string(), SensorTypes::Camera))
            metadata_paths.push_back(cam_path);
        }
      }
    }
    // assure order to be lexicographic
    std::sort(metadata_paths.begin(), metadata_paths.end());
    std::set<int64_t> timestamps;
    for (const auto &metadata_path : metadata_paths) {
      read_camera_metadata(metadata_path, timestamps);
    }
    data->image_timestamps =
        std::vector<int64_t>(timestamps.begin(), timestamps.end());

    const auto imu_path = root / "imu.csv";
    if (fs::is_regular_file(imu_path)) {
      all_sensor_names.push_back("imu");
      if (sensor_filter.keep("imu", SensorTypes::Imu)) read_imu_data(imu_path);
    }

    sensor_filter.warn_unmatched(all_sensor_names);
  }

  void reset() { data.reset(); }

  VioDatasetPtr get_data() { return data; }

 private:
  void read_camera_metadata(const fs::path &path,
                            std::set<int64_t> &timestamps) {
    std::ifstream f(path);
    auto &cam_meta = data->cam_meta.emplace_back();
    cam_meta.name = path.stem().string();
    cam_meta.basepath = path.parent_path() / path.stem();
    std::string line;
    while (std::getline(f, line)) {
      if (line[0] == '#') continue;

      std::stringstream ss(line);

      char tmp;
      std::string filename;
      int64_t timestamp;
      int64_t exposure;

      ss >> timestamp >> tmp >> exposure >> tmp >> filename;

      timestamps.insert(timestamp);
      cam_meta.timestamps.push_back(timestamp);
      cam_meta.exposures.insert({timestamp, exposure * 1e-6});
      cam_meta.filenames.insert({timestamp, filename});
    }
  }

  void read_imu_data(const fs::path &path) {
    std::ifstream f(path);
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

  std::shared_ptr<DaiVioDataset> data;
};  // namespace basalt

}  // namespace basalt

#endif  // DATASET_IO_H
