#pragma once

#include <basalt/calibration/calibration_helper.h>

namespace basalt {

class ResponseEstimator {
 public:
  using Vec256d = Eigen::Vector<double, 256>;

  ResponseEstimator(const VioDatasetPtr &vio_dataset,
                    const Eigen::aligned_vector<Eigen::Vector2i> &resolutions,
                    const std::vector<std::vector<bool>> &mask);

  void compute_error();

  void opt_irradiance();

  void opt_response();

  void rescale();

  void optimize();

  void compute_data_log(std::vector<std::vector<float>> &resp_data_log);

  void save_resp_txt(const std::string &path);

  inline const auto &get_response() { return response; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  const VioDatasetPtr vio_dataset;
  const std::vector<std::vector<bool>> mask;

  Eigen::aligned_vector<Eigen::VectorXd> irradiance;
  Eigen::aligned_vector<Vec256d> response;
};
}  // namespace basalt
