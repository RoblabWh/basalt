#pragma once

#include <basalt/calibration/calibration_helper.h>

namespace basalt {

struct AllanDeviations {
  using Mat6Xd = Eigen::Matrix<double, 6, -1>;
  using Ptr = std::shared_ptr<AllanDeviations>;

  double tau_0;
  Mat6Xd deviations;
};

class AllanVarianceComputor {
  using Vec6d = Eigen::Vector<double, 6>;
  using Mat23d = Eigen::Matrix<double, 2, 3>;
  using CalibrationPtr = Calibration<double>::Ptr;
  using AllanDeviationsPtr = AllanDeviations::Ptr;

  double period_min, period_max;
  uint64_t period_num, period_start;
  double wn_min, wn_max;
  double rr_min, rr_max;

  const VioDatasetPtr vio_dataset;
  CalibrationPtr calib;

  AllanDeviations::Mat6Xd deviations;
  bool computation_complete = false;

  /**
   * Lines for each axis (x, y, z) as (slope, intercept) representing the white
   * noise and random walk for gyroscope and accelerometer.
   */
  Mat23d gyro_wn;
  Mat23d accel_wn;
  Mat23d gyro_rr;
  Mat23d accel_rr;

 public:
  AllanVarianceComputor(const VioDatasetPtr &vio_dataset, CalibrationPtr &calib,
                        AllanDeviationsPtr &init_deviations, double wn_min,
                        double wn_max, double rr_min, double rr_max,
                        double period_min = -1., double period_max = -1.);

  /**
   * Compute allan deviations for each axis of gyroscope and accelerometer.
   */
  void compute_deviations();

  /**
   * Fit lines in the allan plot for the white noise and random walk.
   */
  void fit_lines(double wn_min, double wn_max, double rr_min, double rr_max);

  /**
   * If allen deviation computation is complete.
   */
  bool is_computation_complete() const { return period_start == period_num; }

  /**
   * Compute the log10-log10 plot data for pangolin viewer.
   */
  std::vector<std::vector<float>> compute_data_log() const;

  /**
   * Get copy of current allan deviations with tau_0.
   */
  AllanDeviations::Ptr get_deviations() const {
    return std::make_shared<AllanDeviations>(
        AllanDeviations{period_min, deviations});
  }
};

}  // namespace basalt

namespace cereal {

template <class Archive>
void serialize(Archive &ar, basalt::AllanDeviations &m) {
  ar(m.tau_0);
  ar(m.deviations);
}

}  // namespace cereal
