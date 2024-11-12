
#include <basalt/calibration/allan_variance.h>

#include <tbb/global_control.h>
#include <tbb/parallel_for.h>

namespace basalt {

AllanVarianceComputor::AllanVarianceComputor(const VioDatasetPtr &vio_dataset,
                                             CalibrationPtr &calib)
    : vio_dataset(vio_dataset), calib(calib) {
  allan_deviations.setZero(6, period_num);
}

inline double line_y(const Eigen::Vector2d &line, double x) {
  return std::exp(line[0] * std::log(x) + line[1]);
}

void AllanVarianceComputor::compute() {
  compute_deviations();
  fit_lines();

  for (uint8_t i = 0; i < 3; ++i) {
    calib->gyro_noise_std[i] = line_y(gyro_wn.col(i), 1.0);
    calib->accel_noise_std[i] = line_y(accel_wn.col(i), 1.0);
    calib->gyro_bias_std[i] = line_y(gyro_rr.col(i), 3.0);
    calib->accel_bias_std[i] = line_y(accel_rr.col(i), 3.0);
  }
}

void AllanVarianceComputor::compute_deviations() {
  const auto &gyro_data = vio_dataset->get_gyro_data();
  const auto &accel_data = vio_dataset->get_accel_data();
  assert(accel_data.size() == gyro_data.size());

  tbb::parallel_for(tbb::blocked_range(0u, period_num), [&](const auto &r) {
    for (uint32_t period = r.begin(); period < r.end(); ++period) {
      const double period_time =
          (period + 1) * 0.1;  // Sampling periods from 0.1s to 1000s
      const size_t max_bin_size = period_time * calib->imu_update_rate + .5;
      const size_t averages_num = gyro_data.size() / max_bin_size;
      const size_t averages_num_m1 = averages_num - 1;

      Eigen::aligned_vector<Vec6d> averages(averages_num);

      // Compute Averages
      for (size_t j = 0; j < averages_num; ++j) {
        Vec6d current_average = Vec6d::Zero();

        // get average for current bin
        for (size_t m = 0; m < max_bin_size; ++m) {
          const auto &idx = max_bin_size * j + m;
          current_average.head<3>() += gyro_data[idx].data;
          current_average.tail<3>() += accel_data[idx].data;
        }

        averages[j] = current_average / max_bin_size;
      }

      // Compute Allan Variance
      Vec6d allan_variance = Vec6d::Zero();
      for (size_t k = 0; k < averages_num_m1; ++k) {
        const auto &diff = averages[k + 1] - averages[k];
        allan_variance += diff.array().square().matrix();
      }
      allan_variance /= (2 * averages_num_m1);

      // Compute Allan Deviation
      Vec6d allan_deviation = allan_variance.cwiseSqrt();

      allan_deviations.col(period) = allan_deviation;
    }
  });
}

inline Eigen::Vector2d fit_line(const Eigen::VectorXd &x,
                                const Eigen::VectorXd &y, double m) {
  Eigen::Vector2d line;
  line[0] = m;
  line[1] = (y.array().log() - x.array().log() * m).mean();
  return line;
}

void AllanVarianceComputor::fit_lines() {
  const uint32_t white_noise_break_point = 100;  // 10 secs in deciseconds

  auto period =
      Eigen::VectorXd::LinSpaced(period_num, period_min, period_max) * .1;
  auto gyro_x = allan_deviations.row(0);
  auto gyro_y = allan_deviations.row(1);
  auto gyro_z = allan_deviations.row(2);
  auto accel_x = allan_deviations.row(3);
  auto accel_y = allan_deviations.row(4);
  auto accel_z = allan_deviations.row(5);

  auto period_head = period.head(white_noise_break_point);

  gyro_wn.col(0) =
      fit_line(period_head, gyro_x.head(white_noise_break_point), -0.5);
  gyro_wn.col(1) =
      fit_line(period_head, gyro_y.head(white_noise_break_point), -0.5);
  gyro_wn.col(2) =
      fit_line(period_head, gyro_z.head(white_noise_break_point), -0.5);
  accel_wn.col(0) =
      fit_line(period_head, accel_x.head(white_noise_break_point), -0.5);
  accel_wn.col(1) =
      fit_line(period_head, accel_y.head(white_noise_break_point), -0.5);
  accel_wn.col(2) =
      fit_line(period_head, accel_z.head(white_noise_break_point), -0.5);

  gyro_rr.col(0) = fit_line(period, gyro_x, 0.5);
  gyro_rr.col(1) = fit_line(period, gyro_y, 0.5);
  gyro_rr.col(2) = fit_line(period, gyro_z, 0.5);
  accel_rr.col(0) = fit_line(period, accel_x, 0.5);
  accel_rr.col(1) = fit_line(period, accel_y, 0.5);
  accel_rr.col(2) = fit_line(period, accel_z, 0.5);
}

std::vector<std::vector<float>> AllanVarianceComputor::compute_data_log()
    const {
  std::vector<std::vector<float>> data_log(period_num);

  for (uint32_t period = 0; period < period_num; ++period) {
    std::vector<float> vals;
    vals.reserve(19);
    double period_time = (period + 1) * .1;
    vals.push_back(std::log10(period_time));
    for (uint8_t k = 0; k < 6; ++k)
      vals.push_back(std::log10(allan_deviations.col(period)[k]));
    for (uint8_t k = 0; k < 3; ++k)
      vals.push_back(std::log10(line_y(gyro_wn.col(k), period_time)));
    for (uint8_t k = 0; k < 3; ++k)
      vals.push_back(std::log10(line_y(gyro_rr.col(k), period_time)));
    for (uint8_t k = 0; k < 3; ++k)
      vals.push_back(std::log10(line_y(accel_wn.col(k), period_time)));
    for (uint8_t k = 0; k < 3; ++k)
      vals.push_back(std::log10(line_y(accel_rr.col(k), period_time)));

    data_log[period] = vals;
  }

  return data_log;
}

}  // namespace basalt
