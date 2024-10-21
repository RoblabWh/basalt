
#include <basalt/calibration/allan_variance.h>

#include <tbb/global_control.h>
#include <tbb/parallel_for.h>

namespace basalt {

AllanVarianceComputor::AllanVarianceComputor(const VioDatasetPtr &vio_dataset,
                                             CalibrationPtr &calib)
    : vio_dataset(vio_dataset), calib(calib) {
  allan_deviations.setZero(6, period_num);
}
void AllanVarianceComputor::compute() {
  compute_deviations();
  fit_lines();

  calib->gyro_noise_std = gyro_wn.row(1);
  calib->accel_noise_std = accel_wn.row(1);
  calib->gyro_bias_std = gyro_rr.row(1);
  calib->accel_bias_std = accel_rr.row(1);
}

void AllanVarianceComputor::compute_deviations() {
  const auto &gyro_data = vio_dataset->get_gyro_data();
  const auto &accel_data = vio_dataset->get_accel_data();
  assert(accel_data.size() == gyro_data.size());

  // std::mutex duration_mtx;
  // std::chrono::nanoseconds duration_it(0), duration_avg(0), duration_avar(0), duration_adev(0);

  // auto stp = std::chrono::steady_clock::now();
  tbb::parallel_for(tbb::blocked_range(0u, period_num), [&](const auto &r) {
    for (uint32_t period = r.begin(); period < r.end(); ++period) {
      // auto start_it = std::chrono::steady_clock::now();
      const double period_time =
          (period + 1) * 0.1;  // Sampling periods from 0.1s to 1000s
      const size_t max_bin_size = period_time * calib->imu_update_rate + .5;
      const size_t averages_num = gyro_data.size() / max_bin_size;
      const size_t averages_num_m1 = averages_num - 1;

      Eigen::aligned_vector<Vec6d> averages(averages_num);

      // Compute Averages
      // auto start_avg = std::chrono::steady_clock::now();
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
      // auto end_avg = std::chrono::steady_clock::now();

      // Compute Allan Variance
      // auto start_avar = std::chrono::steady_clock::now();
      Vec6d allan_variance = Vec6d::Zero();
      for (size_t k = 0; k < averages_num_m1; ++k) {
        const auto &diff = averages[k + 1] - averages[k];
        allan_variance += diff.array().square().matrix();
      }
      allan_variance /= (2 * averages_num_m1);
      // auto end_avar = std::chrono::steady_clock::now();

      // Compute Allan Deviation
      // auto start_adev = std::chrono::steady_clock::now();
      Vec6d allan_deviation = allan_variance.cwiseSqrt();
      // auto end_adev = std::chrono::steady_clock::now();

      allan_deviations.col(period) = allan_deviation;

      // auto end_it = std::chrono::steady_clock::now();
      // {
      //   std::lock_guard lock(duration_mtx);
      //   duration_it += end_it - start_it;
      //   duration_avg += end_avg - start_avg;
      //   duration_avar += end_avar - start_avar;
      //   duration_adev += end_adev - start_adev;
      // }
    }
  });
  // auto etp = std::chrono::steady_clock::now();
  // std::cout
  //     << "duration: "
  //     << std::chrono::duration_cast<std::chrono::duration<double>>(etp - stp)
  //            .count()
  //     // << " iteration: " << std::chrono::duration_cast<std::chrono::duration<double>>(duration_it / (period_num)).count()
  //     // << " avarages: " << std::chrono::duration_cast<std::chrono::duration<double>>(duration_avg / (period_num)).count()
  //     // << " variance: " << std::chrono::duration_cast<std::chrono::duration<double>>(duration_avar / (period_num)).count()
  //     // << " deviation: " << std::chrono::duration_cast<std::chrono::duration<double>>(duration_adev / (period_num)).count()
  //     << std::endl;
}

/**
 * a * x + m = y
 * A * x = b (x = {a, m}T)
 */
Eigen::Vector2d fit_line(const Eigen::VectorXd &x, const Eigen::VectorXd &y) {
  Eigen::Matrix2Xd A = Eigen::Matrix2Xd::Ones(2, x.size());
  A.row(0) = x;
  Eigen::VectorXd b = y;

  return A.transpose().colPivHouseholderQr().solve(b);
}
// TODO figure out if log -> exp is better
// Eigen::Vector2d fit_line(const Eigen::VectorXd &x, const Eigen::VectorXd &y) {
//   Eigen::Matrix2Xd A = Eigen::Matrix2Xd::Ones(2, x.size());
//   A.row(0) = x.array().log().matrix();
//   Eigen::VectorXd b = y.array().log().matrix();

//   return A.transpose().colPivHouseholderQr().solve(b).array().exp().matrix();
// }

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

  gyro_wn.col(0) = fit_line(period_head, gyro_x.head(white_noise_break_point));
  gyro_wn.col(1) = fit_line(period_head, gyro_y.head(white_noise_break_point));
  gyro_wn.col(2) = fit_line(period_head, gyro_z.head(white_noise_break_point));
  accel_wn.col(0) =
      fit_line(period_head, accel_x.head(white_noise_break_point));
  accel_wn.col(1) =
      fit_line(period_head, accel_y.head(white_noise_break_point));
  accel_wn.col(2) =
      fit_line(period_head, accel_z.head(white_noise_break_point));

  gyro_rr.col(0) = fit_line(period, gyro_x);
  gyro_rr.col(1) = fit_line(period, gyro_y);
  gyro_rr.col(2) = fit_line(period, gyro_z);
  accel_rr.col(0) = fit_line(period, accel_x);
  accel_rr.col(1) = fit_line(period, accel_y);
  accel_rr.col(2) = fit_line(period, accel_z);
}

std::vector<std::vector<float>> AllanVarianceComputor::compute_data_log()
    const {
  std::vector<std::vector<float>> data_log(period_num);

  for (uint32_t period = 0; period < period_num; ++period) {
    std::vector<float> vals;
    vals.reserve(19);
    double period_time = (period + 1) * .1;
    vals.push_back(period_time);
    for (uint8_t k = 0; k < 6; ++k)
      vals.push_back(allan_deviations.col(period)[k]);
    for (uint8_t k = 0; k < 3; ++k)
      vals.push_back(gyro_wn.col(k)[0] * period_time + gyro_wn.col(k)[1]);
    for (uint8_t k = 0; k < 3; ++k)
      vals.push_back(gyro_rr.col(k)[0] * period_time + gyro_rr.col(k)[1]);
    for (uint8_t k = 0; k < 3; ++k)
      vals.push_back(accel_wn.col(k)[0] * period_time + accel_wn.col(k)[1]);
    for (uint8_t k = 0; k < 3; ++k)
      vals.push_back(accel_rr.col(k)[0] * period_time + accel_rr.col(k)[1]);

    data_log[period] = vals;
  }

  return data_log;
}

}  // namespace basalt
