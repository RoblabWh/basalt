
#include <basalt/calibration/allan_variance.h>

#include <tbb/global_control.h>
#include <tbb/parallel_for.h>

namespace basalt {

AllanVarianceComputor::AllanVarianceComputor(
    const VioDatasetPtr &vio_dataset, CalibrationPtr &calib,
    AllanDeviationsPtr &init_deviations, double wn_min, double wn_max,
    double rr_min, double rr_max, double period_min, double period_max)
    : period_min(period_min),
      period_max(period_max),
      wn_min(wn_min),
      wn_max(wn_max),
      rr_min(rr_min),
      rr_max(rr_max),
      vio_dataset(vio_dataset),
      calib(calib) {
  if (period_min < 0) period_min = std::min(wn_min, rr_min);
  if (period_max < 0) period_max = std::max(wn_max, rr_max);
  period_start = 0;
  period_num = period_max / period_min + .5;
  deviations.setConstant(6, period_num,
                         std::numeric_limits<double>::quiet_NaN());
  if (init_deviations) {
    if (period_min == init_deviations->tau_0) {
      period_start =
          std::min(static_cast<uint64_t>(init_deviations->deviations.cols()),
                   period_num);
      deviations.leftCols(period_start) =
          init_deviations->deviations.leftCols(period_start);
      if (period_start < period_num) {
        std::cout << "Allan deviations pre-processed up to " << period_start
                  << "/" << period_num << ", need to calculate the remaining."
                  << std::endl;
      }
    } else {
      std::cout
          << "Allan deviations tau_0 does not match, recompute with new tau_0"
          << std::endl;
    }
    gyro_wn.setConstant(std::numeric_limits<double>::quiet_NaN());
    gyro_rr.setConstant(std::numeric_limits<double>::quiet_NaN());
    accel_wn.setConstant(std::numeric_limits<double>::quiet_NaN());
    accel_rr.setConstant(std::numeric_limits<double>::quiet_NaN());
  }

  assert(wn_min < wn_max);
  assert(rr_min < rr_max);
  assert(period_min <= wn_min);
  assert(period_min <= rr_min);
  assert(period_max >= wn_max);
  assert(period_max >= rr_max);
}

void AllanVarianceComputor::compute_deviations() {
  const auto &gyro_data = vio_dataset->get_gyro_data();
  const auto &accel_data = vio_dataset->get_accel_data();
  assert(accel_data.size() == gyro_data.size());

  std::mutex mtx;
  tbb::parallel_for(
      tbb::blocked_range(period_start, period_num), [&](const auto &r) {
        for (uint32_t period = r.begin(); period < r.end(); ++period) {
          const double period_time = (period + 1) * period_min;
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

          deviations.col(period) = allan_deviation;

          {
            // Print progress
            std::lock_guard<std::mutex> lock(mtx);
            ++period_start;
            if (period_start % 1000 == 0) {
              std::cout << "Progress: " << period_start << "/" << period_num
                        << " - " << std::setprecision(2) << std::fixed
                        << period_start * 100.0 / period_num << '%'
                        << std::endl;
            }
          }
        }
      });
}

inline double line_y(const Eigen::Vector2d &line, double x) {
  return std::exp(line[0] * std::log(x) + line[1]);
}

inline Eigen::Vector2d fit_line(const Eigen::VectorXd &x,
                                const Eigen::VectorXd &y, double m) {
  Eigen::Vector2d line;
  line[0] = m;
  line[1] = (y.array().log() - x.array().log() * m).mean();
  return line;
}

void AllanVarianceComputor::fit_lines(double wn_min, double wn_max,
                                      double rr_min, double rr_max) {
  const uint32_t wn_start = wn_min / period_min - 0.5;
  const uint32_t wn_end = wn_max / period_min + 0.5;
  const uint32_t rr_start = rr_min / period_min - 0.5;
  const uint32_t rr_end = rr_max / period_min + 0.5;
  const uint32_t wn_size = wn_end - wn_start;
  const uint32_t rr_size = rr_end - rr_start;

  const auto period =
      Eigen::VectorXd::LinSpaced(period_num, 1, period_num) * period_min;
  const auto gyro_x = deviations.row(0);
  const auto gyro_y = deviations.row(1);
  const auto gyro_z = deviations.row(2);
  const auto accel_x = deviations.row(3);
  const auto accel_y = deviations.row(4);
  const auto accel_z = deviations.row(5);

  const auto wn_period = period.segment(wn_start, wn_size);
  const auto rr_period = period.segment(rr_start, rr_size);

  gyro_wn.col(0) = fit_line(wn_period, gyro_x.segment(wn_start, wn_size), -0.5);
  gyro_wn.col(1) = fit_line(wn_period, gyro_y.segment(wn_start, wn_size), -0.5);
  gyro_wn.col(2) = fit_line(wn_period, gyro_z.segment(wn_start, wn_size), -0.5);
  accel_wn.col(0) =
      fit_line(wn_period, accel_x.segment(wn_start, wn_size), -0.5);
  accel_wn.col(1) =
      fit_line(wn_period, accel_y.segment(wn_start, wn_size), -0.5);
  accel_wn.col(2) =
      fit_line(wn_period, accel_z.segment(wn_start, wn_size), -0.5);

  gyro_rr.col(0) = fit_line(rr_period, gyro_x.segment(rr_start, rr_size), 0.5);
  gyro_rr.col(1) = fit_line(rr_period, gyro_y.segment(rr_start, rr_size), 0.5);
  gyro_rr.col(2) = fit_line(rr_period, gyro_z.segment(rr_start, rr_size), 0.5);
  accel_rr.col(0) =
      fit_line(rr_period, accel_x.segment(rr_start, rr_size), 0.5);
  accel_rr.col(1) =
      fit_line(rr_period, accel_y.segment(rr_start, rr_size), 0.5);
  accel_rr.col(2) =
      fit_line(rr_period, accel_z.segment(rr_start, rr_size), 0.5);

  for (uint8_t i = 0; i < 3; ++i) {
    calib->gyro_noise_std[i] = line_y(gyro_wn.col(i), 1.0);
    calib->accel_noise_std[i] = line_y(accel_wn.col(i), 1.0);
    calib->gyro_bias_std[i] = line_y(gyro_rr.col(i), 3.0);
    calib->accel_bias_std[i] = line_y(accel_rr.col(i), 3.0);
  }
}

std::vector<std::vector<float>> AllanVarianceComputor::compute_data_log()
    const {
  std::vector<std::vector<float>> data_log(period_num);

  for (uint32_t period = 0; period < period_num; ++period) {
    std::vector<float> vals;
    vals.reserve(19);
    double period_time = (period + 1) * period_min;
    vals.push_back(std::log10(period_time));
    for (uint8_t k = 0; k < 6; ++k)
      vals.push_back(std::log10(deviations.col(period)[k]));
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
