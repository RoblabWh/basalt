#include <basalt/calibration/response.h>

#include <tbb/blocked_range.h>
#include <tbb/combinable.h>
#include <tbb/parallel_for.h>

namespace basalt {

ResponseEstimator::ResponseEstimator(
    const VioDatasetPtr &vio_dataset,
    const Eigen::aligned_vector<Eigen::Vector2i> &resolutions,
    const std::vector<std::vector<bool>> &mask)
    : vio_dataset(vio_dataset), mask(mask) {
  const auto num_cams = vio_dataset->get_num_cams();
  response.resize(num_cams);
  irradiance.resize(num_cams);
  for (size_t i = 0; i < num_cams; ++i) {
    const auto resolution = resolutions[i];
    response[i].setLinSpaced(0, response[i].size() - 1);
    irradiance[i].setOnes(resolution[0] * resolution[1]);
  }
}

void ResponseEstimator::compute_error() {
  tbb::combinable<Eigen::VectorX<long double>> sum_error(
      Eigen::VectorX<long double>::Zero(vio_dataset->get_num_cams()));
  tbb::combinable<Eigen::VectorX<long double>> num_residuals(
      Eigen::VectorX<long double>::Zero(vio_dataset->get_num_cams()));

  const auto &timestamps = vio_dataset->get_image_timestamps();

  tbb::parallel_for(
      tbb::blocked_range(0ul, timestamps.size()), [&](const auto &r) {
        auto &error_part = sum_error.local();
        auto &num_residuals_part = num_residuals.local();
        for (auto stamp_i = r.begin(); stamp_i < r.end(); ++stamp_i) {
          const auto data_vec =
              vio_dataset->get_image_data(timestamps[stamp_i]);
          for (size_t cam_i = 0; cam_i < data_vec.size(); ++cam_i) {
            const auto data = data_vec[cam_i];
            for (size_t i = 0; i < data.img->size(); ++i) {
              if (!mask[cam_i][i]) continue;
              const auto p = (*data.img)[i] >> 8;
              if (p == 255) continue;
              long double residual =
                  response[cam_i][p] - data.exposure * irradiance[cam_i][i];
              if (!std::isfinite(residual)) continue;
              error_part[cam_i] += residual * residual * 1e-10;
              num_residuals_part[cam_i]++;
            }
          }
        }
      });
  Eigen::VectorX<long double> error =
      1e5 * sum_error.combine(std::plus())
                .cwiseQuotient(num_residuals.combine(std::plus()))
                .cwiseSqrt();
  std::cout << std::fixed << "error: "
            << error.transpose()
            // << " sum: " << sum_error.combine(std::plus()).transpose()
            // << " num: " << num_residuals.combine(std::plus()).transpose()
            << std::endl;
}

template <typename T>
inline Eigen::aligned_vector<T> eigen_aligned_vector_cwise_add(
    const Eigen::aligned_vector<T> &a, const Eigen::aligned_vector<T> &b) {
  Eigen::aligned_vector<T> c(a.size());
  for (size_t i = 0; i < c.size(); ++i) c[i] = a[i] + b[i];
  return c;
}

void ResponseEstimator::opt_irradiance() {
  Eigen::aligned_vector<Eigen::VectorXd> init_irradiance(irradiance.size());
  for (size_t i = 0; i < irradiance.size(); ++i)
    init_irradiance[i].setZero(irradiance[i].size());
  tbb::combinable<Eigen::aligned_vector<Eigen::VectorXd>> new_irradiance_pool(
      init_irradiance);
  tbb::combinable<Eigen::aligned_vector<Eigen::VectorXd>>
      new_irradiance_count_pool(init_irradiance);

  const auto &timestamps = vio_dataset->get_image_timestamps();

  tbb::parallel_for(
      tbb::blocked_range(0ul, timestamps.size()), [&](const auto &r) {
        auto &new_irradiance_part = new_irradiance_pool.local();
        auto &new_irradiance_count_part = new_irradiance_count_pool.local();
        for (auto stamp_i = r.begin(); stamp_i < r.end(); ++stamp_i) {
          const auto data_vec =
              vio_dataset->get_image_data(timestamps[stamp_i]);
          for (size_t cam_i = 0; cam_i < data_vec.size(); ++cam_i) {
            const auto data = data_vec[cam_i];
            for (size_t i = 0; i < data.img->size(); ++i) {
              if (!mask[cam_i][i]) continue;
              const auto p = (*data.img)[i] >> 8;
              if (p == 255) continue;
              new_irradiance_part[cam_i][i] +=
                  response[cam_i][p] * data.exposure;
              new_irradiance_count_part[cam_i][i] +=
                  data.exposure * data.exposure;
            }
          }
        }
      });

  const auto new_irradiance = new_irradiance_pool.combine(
      eigen_aligned_vector_cwise_add<Eigen::VectorXd>);
  const auto new_irradiance_count = new_irradiance_count_pool.combine(
      eigen_aligned_vector_cwise_add<Eigen::VectorXd>);

  for (size_t cam_i = 0; cam_i < irradiance.size(); ++cam_i) {
    for (Eigen::Index i = 0; i < irradiance[cam_i].size(); i++) {
      if (new_irradiance_count[cam_i][i] > 0)
        irradiance[cam_i][i] =
            new_irradiance[cam_i][i] / new_irradiance_count[cam_i][i];
    }
  }
}

void ResponseEstimator::opt_response() {
  tbb::combinable<Eigen::aligned_vector<Vec256d>> new_response_pool(
      Eigen::aligned_vector<Vec256d>(response.size(), Vec256d::Zero()));
  tbb::combinable<Eigen::aligned_vector<Vec256d>> new_response_count_pool(
      Eigen::aligned_vector<Vec256d>(response.size(), Vec256d::Zero()));

  const auto &timestamps = vio_dataset->get_image_timestamps();

  tbb::parallel_for(
      tbb::blocked_range(0ul, timestamps.size()), [&](const auto &r) {
        auto &new_response_part = new_response_pool.local();
        auto &new_response_count_part = new_response_count_pool.local();
        for (auto stamp_i = r.begin(); stamp_i < r.end(); ++stamp_i) {
          const auto data_vec =
              vio_dataset->get_image_data(timestamps[stamp_i]);
          for (size_t cam_i = 0; cam_i < data_vec.size(); ++cam_i) {
            const auto data = data_vec[cam_i];
            for (size_t i = 0; i < data.img->size(); ++i) {
              if (!mask[cam_i][i]) continue;
              const auto p = (*data.img)[i] >> 8;
              if (p == 255) continue;
              new_response_part[cam_i][p] +=
                  irradiance[cam_i][i] * data.exposure;
              new_response_count_part[cam_i][p]++;
            }
          }
        }
      });

  const auto new_response =
      new_response_pool.combine(eigen_aligned_vector_cwise_add<Vec256d>);
  const auto new_response_count =
      new_response_count_pool.combine(eigen_aligned_vector_cwise_add<Vec256d>);

  for (size_t cam_i = 0; cam_i < response.size(); ++cam_i) {
    response[cam_i] =
        new_response[cam_i].cwiseQuotient(new_response_count[cam_i]);
    for (uint16_t i = 0; i < 256; ++i) {
      if (!std::isfinite(response[cam_i][i]) && i > 1)
        response[cam_i][i] = response[cam_i][i - 1] +
                             (response[cam_i][i - 1] - response[cam_i][i - 2]);
    }
  }
}

void ResponseEstimator::rescale() {
  for (size_t i = 0; i < response.size(); ++i) {
    double rescale_factor = 255.0 / response[i][255];
    // std::cout << "rescale factor: " << rescale_factor << std::endl;
    irradiance[i] *= rescale_factor;
    response[i] *= rescale_factor;
  }
}

void ResponseEstimator::optimize() {
  // compute_error();
  for (int i = 0; i < 10; i++) {
    opt_irradiance();
    // compute_error();
    opt_response();
    // compute_error();
    rescale();
    // compute_error();
  }
}

void ResponseEstimator::compute_data_log(
    std::vector<std::vector<float>> &resp_data_log) {
  resp_data_log.resize(response.front().size());
  for (size_t i = 0; i < resp_data_log.size(); ++i) {
    resp_data_log[i].resize(response.size());
    for (size_t cam_i = 0; cam_i < response.size(); ++cam_i)
      resp_data_log[i][cam_i] = response[cam_i][i];
  }
}

}  // namespace basalt
