#pragma once

#include <basalt/calibration/calibration_helper.h>

namespace basalt {

class AllanVarianceComputor {
  using Vec6d = Eigen::Vector<double, 6>;
  using Mat23d = Eigen::Matrix<double, 2, 3>;
  using Mat6Xd = Eigen::Matrix<double, 6, -1>;
  using CalibrationPtr = Calibration<double>::Ptr;

  double period_min, period_max;
  uint32_t period_num;
  double wn_min, wn_max;
  double rr_min, rr_max;

  const VioDatasetPtr vio_dataset;
  CalibrationPtr calib;

  Mat6Xd allan_deviations;

  /**
   * Lines for each axis (x, y, z) as (slope, intercept) representing the white
   * noise and random walk for gyroscope and accelerometer.
   */
  Mat23d gyro_wn;
  Mat23d accel_wn;
  Mat23d gyro_rr;
  Mat23d accel_rr;

 public:
  AllanVarianceComputor(const VioDatasetPtr &vio_dataset,
                        CalibrationPtr &calib,
                        double wn_min, double wn_max,
                        double rr_min, double rr_max,
                        double period_min = -1., double period_max = -1.);

  /**
   * Compute white noise and random walk for each axis of gyroscope and
   * accelerometer and write result to calibration.
   */
  void compute();

  /**
   * Compute the log10-log10 plot data for pangolin viewer.
   */
  std::vector<std::vector<float>> compute_data_log() const;

 private:
  void compute_deviations();
  void fit_lines();
};

}  // namespace basalt
