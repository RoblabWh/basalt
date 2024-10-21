#pragma once

#include <basalt/calibration/calibration_helper.h>

namespace basalt {

class AllanVarianceComputor {
  using Vec6d = Eigen::Vector<double, 6>;
  using Mat23d = Eigen::Matrix<double, 2, 3>;
  using Mat6Xd = Eigen::Matrix<double, 6, -1>;
  using CalibrationPtr = Calibration<double>::Ptr;

  // Range we will sample from (0.1s to 1000s)
  const uint32_t period_min = 1;
  const uint32_t period_max = 10000;
  const uint32_t period_num = period_max - period_min + 1;

  const VioDatasetPtr vio_dataset;
  CalibrationPtr calib;

  Mat6Xd allan_deviations;

  Mat23d gyro_wn;
  Mat23d accel_wn;
  Mat23d gyro_rr;
  Mat23d accel_rr;

 public:
  AllanVarianceComputor(const VioDatasetPtr &vio_dataset,
                        CalibrationPtr &calib);

  void compute();
  std::vector<std::vector<float>> compute_data_log() const;

 private:
  void compute_deviations();
  void fit_lines();
};

template <class T>
inline uint64_t calculateCaptureTimeNs(const Eigen::aligned_vector<T> &data) {
  return data.back().timestamp_ns - data.front().timestamp_ns;
}

template <class T>
inline double calculateRate(const Eigen::aligned_vector<T> &data) {
  const uint64_t duration = calculateCaptureTimeNs(data);
  const double dt = duration / (data.size() - 1.);
  return 1e9 / dt;
}

}  // namespace basalt
