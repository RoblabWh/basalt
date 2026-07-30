/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once

#include <pangolin/display/image_view.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/image/image.h>
#include <pangolin/image/image_io.h>
#include <pangolin/image/typed_image.h>
#include <pangolin/pangolin.h>

#include <Eigen/Dense>

#include <iostream>
#include <limits>
#include <thread>

#include <basalt/calibration/aprilgrid.h>
#include <basalt/calibration/calib_cache.h>
#include <basalt/calibration/calibration_helper.h>
#include <basalt/image/image.h>
#include <basalt/utils/test_utils.h>
#include <basalt/utils/sophus_utils.hpp>

namespace basalt {

class PosesOptimization;

struct CamCalibOptions {
  std::string dataset_path;
  std::string dataset_type;
  std::string aprilgrid_path;
  std::string cache_path;
  std::string cache_dataset_name;
  std::vector<std::string> cam_types;
  std::vector<std::string> sensor_include;
  std::vector<std::string> sensor_exclude;
  int skip_images = 1;
  int start_image = 0;
  int end_image = 0;
  bool opt_intr = true;
  double huber_thresh = 4.0;
  double stop_thresh = 1e-8;
  bool show_gui = true;
};

class CamCalib {
 public:
  explicit CamCalib(const CamCalibOptions &options);

  ~CamCalib();

  void initGui();

  void computeVign();

  void computeResp();

  void setLinearResp();

  void setNumCameras(size_t n);

  void renderingLoop();

  int runHeadless(int max_iterations, bool compute_vignette,
                  bool compute_response);

  void computeProjections();

  void detectCorners();

  void initCamIntrinsics();

  void initCamPoses();

  void initCamExtrinsics();

  void initOptimization();

  void loadDataset();

  void optimize();

  bool optimizeWithParam(bool print_info,
                         std::map<std::string, double> *stats = nullptr);

  void saveCalib();

  void drawImageOverlay(pangolin::View &v, size_t cam_id);

  bool hasCorners() const;

 private:
  bool cornersComplete() const;

  std::vector<std::vector<size_t>> computeCamOverlaps() const;

  static constexpr int UI_WIDTH = 300;

  static constexpr size_t RANSAC_THRESHOLD = 10;

  // typedef Calibration::Ptr CalibrationPtr;

  VioDatasetPtr vio_dataset;
  // CalibrationPtr calib;

  CalibCornerMap calib_corners;
  CalibCornerMap calib_corners_rejected;
  CalibInitPoseMap calib_init_poses;

  // cached data of currently deselected cameras (see calib_cache.h)
  DormantCorners dormant_corners;
  DormantPoses dormant_poses;

  // Per-camera result of initCamExtrinsics; cameras that stay false would
  // enter the optimization with identity extrinsics and derail it.
  std::vector<bool> extrinsics_initialized;

  std::shared_ptr<std::thread> processing_thread;

  std::shared_ptr<PosesOptimization> calib_opt;

  std::map<TimeCamId, ProjectedCornerData> reprojected_corners;
  std::map<TimeCamId, ProjectedCornerData> reprojected_vignette;
  std::map<TimeCamId, std::vector<double>> reprojected_vignette_error;

  std::string dataset_path;
  std::string dataset_type;

  SensorFilter sensor_filter;

  AprilGrid april_grid;

  std::string cache_path;
  std::string cache_dataset_name;

  std::vector<std::string> cam_types;

  int skip_images;
  int start_image;
  int end_image;

  bool show_gui;

  const size_t MIN_CORNERS = 15;

  //////////////////////
  pangolin::Var<int> show_frame;
  pangolin::Var<bool> show_ids;
  pangolin::Var<std::function<void(void)>> load_dataset;
  pangolin::Var<bool> show_corners;
  pangolin::Var<bool> show_corners_rejected;
  pangolin::Var<std::function<void(void)>> detect_corners;
  pangolin::Var<std::function<void(void)>> init_cam_intrinsics;
  pangolin::Var<bool> show_init_reproj;
  pangolin::Var<std::function<void(void)>> init_cam_poses;
  pangolin::Var<std::function<void(void)>> init_cam_extrinsics;
  pangolin::Var<bool> show_opt;
  pangolin::Var<bool> show_vign;
  pangolin::Var<std::function<void(void)>> init_opt;
  pangolin::Var<bool> opt_intr;
  pangolin::Var<double> huber_thresh;
  pangolin::Var<double> stop_thresh;
  pangolin::Var<std::function<void(void)>> opt;
  pangolin::Var<bool> opt_until_convg;
  pangolin::Var<bool> vign_cutoff;
  pangolin::Var<std::function<void(void)>> compute_vign;
  pangolin::Var<std::function<void(void)>> save_calib;
  pangolin::Var<std::function<void(void)>> set_lin_resp;
  pangolin::Var<std::function<void(void)>> compute_resp;

  std::shared_ptr<pangolin::Plotter> polar_plotter;
  std::shared_ptr<pangolin::Plotter> azimuth_plotter;
  std::shared_ptr<pangolin::Plotter> vign_plotter;
  std::shared_ptr<pangolin::Plotter> resp_plotter;

  std::vector<pangolin::Colour> cam_colors;

  pangolin::View *img_view_display;

  std::vector<std::shared_ptr<pangolin::ImageView>> img_view;

  pangolin::DataLog vign_data_log, resp_data_log;

  std::vector<std::shared_ptr<pangolin::DataLog>> polar_data_log,
      azimuth_data_log;
};

}  // namespace basalt
