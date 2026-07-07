#ifndef DATASET_IO_CV_H
#define DATASET_IO_CV_H

#include <basalt/io/dataset_io.h>
#include <basalt/utils/filesystem.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace basalt {

//  ONLY use this if no other dataset type works. This is the most limited
//  dataset type there is, which also inherits the worst performance panilty.

class CvVioDataset : public VioDataset {
  std::vector<cv::VideoCapture> captures;
  std::mutex capture_lock;
  int64_t frame_pd;
  std::vector<int64_t> image_timestamps;

 public:
  ~CvVioDataset() {};

  size_t get_num_cams() const { return captures.size(); }

  std::vector<int64_t> &get_image_timestamps() { return image_timestamps; }
  Eigen::aligned_vector<AccelData> &get_accel_data() {
    static Eigen::aligned_vector<AccelData> none;
    return none;
  }
  Eigen::aligned_vector<GyroData> &get_gyro_data() {
    static Eigen::aligned_vector<GyroData> none;
    return none;
  }

  const std::vector<int64_t> &get_gt_timestamps() const {
    static const std::vector<int64_t> none;
    return none;
  }
  const Eigen::aligned_vector<Sophus::SE3d> &get_gt_pose_data() const {
    static const Eigen::aligned_vector<Sophus::SE3d> none;
    return none;
  }

  int64_t get_mocap_to_imu_offset_ns() const { return 0; }

  std::vector<ImageData> get_image_data(int64_t timestamp) {
    std::vector<ImageData> res;
    res.reserve(captures.size());

    uint64_t frame_idx = timestamp / frame_pd;

    std::vector<cv::Mat> imgs;
    imgs.reserve(captures.size());
    {
      std::lock_guard<std::mutex> lock(capture_lock);
      for (auto &capture : captures) {
        cv::Mat &img = imgs.emplace_back();
        if (!capture.set(cv::CAP_PROP_POS_FRAMES, frame_idx)) {
          std::cerr << "Failed to seek image" << std::endl;
          std::abort();
        }
        if (!capture.read(img)) {
          std::cerr << "Failed to read image at timestamp " << timestamp
                    << " (index: " << frame_idx << ")" << std::endl;
          std::abort();
        }
      }
    }

    for (auto &img : imgs) {
      if (img.empty()) {
        std::cerr << "Image at timestamp " << timestamp
                  << " (index: " << frame_idx << ") is empty" << std::endl;
        std::abort();
      }

      if (img.type() == CV_8UC3 || img.type() == CV_16UC3) {
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
      } else if (img.type() == CV_8UC4 || img.type() == CV_16UC4) {
        cv::cvtColor(img, img, cv::COLOR_BGRA2GRAY);
      }

      auto &resimg = res.emplace_back();
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

      resimg.exposure = -1;
    }
    return res;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  friend class CvIO;
};

class CvIO : public DatasetIoInterface {
 public:
  CvIO() {}

  void read(const std::string &path) {
    data.reset(new CvVioDataset);

    std::vector<std::string> paths;
    size_t idx = 0, idx_last = 0;
    while (idx != std::string::npos) {
      idx = path.find(',', idx_last);
      paths.push_back(path.substr(idx_last, idx - idx_last));
      idx_last = idx + 1;
    }

    data->captures.resize(paths.size());
    for (size_t i = 0; i < paths.size(); ++i) {
      if (!data->captures[i].open(paths[i])) {
        std::cerr << "Failed to open \"" << paths[i] << '"';
        std::abort();
      }
    }

    double frame_rate = data->captures.front().get(cv::CAP_PROP_FPS);
    data->frame_pd = 1.0 / frame_rate * 1e9;
    uint64_t frame_num_min =
        data->captures.front().get(cv::CAP_PROP_FRAME_COUNT);

    for (const auto &capture : data->captures) {
      uint64_t frame_num = capture.get(cv::CAP_PROP_FRAME_COUNT);
      if (frame_num < frame_num_min) {
        std::cerr << "Number of frame in input videos differ by "
                  << frame_num_min - frame_num << ". Using lower number "
                  << frame_num << std::endl;
        frame_num_min = frame_num;
      }
      if (capture.get(cv::CAP_PROP_FPS) != frame_rate) {
        std::cerr << "Frame rate of input videos differ" << std::endl;
        std::abort();
      }
    }

    data->image_timestamps.resize(frame_num_min);
    for (uint64_t i = 0; i < frame_num_min; ++i)
      data->image_timestamps[i] = data->frame_pd * i;
  }

  void reset() { data.reset(); }

  VioDatasetPtr get_data() { return data; }

 private:
  std::shared_ptr<CvVioDataset> data;
};  // namespace basalt

}  // namespace basalt

#endif  // DATASET_IO_H
