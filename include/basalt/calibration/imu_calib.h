#pragma once

#include <pangolin/gl/gldraw.h>
#include <pangolin/pangolin.h>

#include <Eigen/Dense>

#include <iostream>
#include <limits>

#include <basalt/calibration/calibration_helper.h>
#include <basalt/utils/test_utils.h>

namespace basalt {

class AllanVarianceComputor;
class AllanDeviations;

class ImuCalib {
 public:
  ImuCalib(const std::string &dataset_path, const std::string &dataset_type,
           const std::string &cache_path, const std::string &cache_dataset_name,
           double wn_min, double wn_max, double rr_min, double rr_max,
           double period_min, double period_max, bool show_gui = true);

  ~ImuCalib();

  void initGui();

  void renderingLoop();

  void loadDataset();

  void compute();

  void fitLines();

  void saveCalib();

  void recomputeDataLog();

  void drawPlots();

 private:
  static constexpr int UI_WIDTH = 300;

  VioDatasetPtr vio_dataset;

  std::unique_ptr<AllanVarianceComputor> avar_computor;
  std::shared_ptr<Calibration<double>> calib;

  std::string dataset_path;
  std::string dataset_type;

  std::string cache_path;
  std::string cache_dataset_name;

  std::shared_ptr<AllanDeviations> allan_deviations;

  double period_min, period_max;

  bool show_gui;

  //////////////////////

  pangolin::Var<bool> show_data;
  pangolin::Var<bool> show_accel;
  pangolin::Var<bool> show_gyro;
  pangolin::Var<bool> show_wn;
  pangolin::Var<bool> show_rr;
  pangolin::Var<std::function<void(void)>> load_dataset;
  pangolin::Var<std::function<void(void)>> comp;
  pangolin::Var<double> wn_min, wn_max;
  pangolin::Var<double> rr_min, rr_max;
  pangolin::Var<std::function<void(void)>> fit;
  pangolin::Var<std::function<void(void)>> save_calib;

  pangolin::Plotter *plotter_raw, *plotter_calib;

  pangolin::DataLog imu_raw_log, imu_calib_log;
  double imu_raw_time_max;
  double imu_raw_gyro_min, imu_raw_gyro_max;
  double imu_raw_accel_min, imu_raw_accel_max;
  float imu_calib_gyro_min, imu_calib_gyro_max;
  float imu_calib_accel_min, imu_calib_accel_max;
};

}  // namespace basalt
