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

#include <basalt/calibration/cam_imu_calib.h>

#include <basalt/utils/system_utils.h>

#include <basalt/serialization/headers_serialization.h>

#include <basalt/optimization/spline_optimize.h>

#include <algorithm>
#include <unordered_set>

namespace basalt {

CamImuCalib::CamImuCalib(const CamImuCalibOptions &options)
    : dataset_path(options.dataset_path),
      dataset_type(options.dataset_type),
      april_grid(options.aprilgrid_path),
      cache_path(ensure_trailing_slash(options.cache_path)),
      cache_dataset_name(options.cache_dataset_name),
      skip_images(options.skip_images),
      start_image(options.start_image),
      end_image(options.end_image),
      show_gui(options.show_gui),
      imu_noise({options.accel_noise_std, options.gyro_noise_std,
                 options.accel_bias_std, options.gyro_bias_std}),
      show_frame("ui.show_frame", 0, 0, 1500),
      show_ids("ui.show_ids", false, false, true),
      show_data("ui.show_data", true, false, true),
      show_accel("ui.show_accel", true, false, true),
      show_gyro("ui.show_gyro", false, false, true),
      load_dataset(
        "ui.load_dataset", std::bind(&CamImuCalib::loadDataset, this)),
      show_corners("ui.show_corners", true, false, true),
      show_corners_rejected("ui.show_corners_rejected", false, false, true),
      detect_corners("ui.detect_corners", std::bind(&CamImuCalib::detectCorners, this)),
      show_init_reproj("ui.show_init_reproj", false, false, true),
      init_cam_poses(
        "ui.init_cam_poses", std::bind(&CamImuCalib::initCamPoses, this)),
        init_cam_imu(
          "ui.init_cam_imu", std::bind(&CamImuCalib::initCamImuTransform, this)),
      show_opt("ui.show_opt", true, false, true),
      show_spline("ui.show_spline", true, false, true),
      show_pos("ui.show_pos", false, false, true),
      show_rot_error("ui.show_rot_error", false, false, true),
      init_opt(
        "ui.init_opt", std::bind(&CamImuCalib::initOptimization, this)),
      opt_intr("ui.opt_intr", options.opt_intr, false, true),
      opt_poses("ui.opt_poses", options.opt_poses, false, true),
      opt_corners("ui.opt_corners", true, false, true),
      opt_cam_time_offset(
        "ui.opt_cam_time_offset", options.opt_cam_time_offset, false, true),
      opt_imu_scale("ui.opt_imu_scale", options.opt_imu_scale, false, true),
      huber_thresh("ui.huber_thresh", options.huber_thresh, 0.1, 10.0),
      stop_thresh("ui.stop_thresh", options.stop_thresh, 1e-10, 0.01, true),
      opt(
        "ui.optimize", std::bind(&CamImuCalib::optimize, this)),
      opt_until_convg("ui.opt_until_converge", false, false, true),
      save_calib(
        "ui.save_calib", std::bind(&CamImuCalib::saveCalib, this)),
      show_mocap("ui.show_mocap", false, false, true),
      show_mocap_rot_error("ui.show_mocap_rot_error", false, false, true),
      show_mocap_rot_vel("ui.show_mocap_rot_vel", false, false, true),
      opt_mocap("ui.opt_mocap", options.opt_mocap, false, true),
      init_mocap(
        "ui.init_mocap", std::bind(&CamImuCalib::initMocap, this)),
      save_mocap_calib(
        "ui.save_mocap_calib", std::bind(&CamImuCalib::saveMocapCalib, this)) {
  if (show_gui) initGui();
}

CamImuCalib::~CamImuCalib() {
  if (processing_thread) {
    processing_thread->join();
  }
}

void CamImuCalib::initGui() {
  pangolin::CreateWindowAndBind("Main", 1600, 1000);

  img_view_display =
      &pangolin::CreateDisplay()
           .SetBounds(0.5, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0)
           .SetLayout(pangolin::LayoutEqual);

  pangolin::View &plot_display = pangolin::CreateDisplay().SetBounds(
      0.0, 0.5, pangolin::Attach::Pix(UI_WIDTH), 1.0);

  pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0,
                                        pangolin::Attach::Pix(UI_WIDTH));

  plotter = new pangolin::Plotter(&imu_data_log, 0.0, 100.0, -1.0, 1.0, 0.01f,
                                  0.01f);
  plot_display.AddDisplay(*plotter);

  setNumCameras(1);
}

void CamImuCalib::setNumCameras(size_t n) {
  while (img_view.size() < n && show_gui) {
    std::shared_ptr<pangolin::ImageView> iv(new pangolin::ImageView);

    size_t idx = img_view.size();
    img_view.push_back(iv);

    img_view_display->AddDisplay(*iv);
    iv->extern_draw_function = std::bind(&CamImuCalib::drawImageOverlay, this,
                                         std::placeholders::_1, idx);
  }
}

void CamImuCalib::renderingLoop() {
  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (vio_dataset) {
      if (show_frame.GuiChanged()) {
        size_t frame_id = static_cast<size_t>(show_frame);
        int64_t timestamp = vio_dataset->get_image_timestamps()[frame_id];

        const std::vector<ImageData> &img_vec =
            vio_dataset->get_image_data(timestamp);

        for (size_t cam_id = 0; cam_id < vio_dataset->get_num_cams(); cam_id++)
          if (img_vec[cam_id].img) {
            pangolin::GlPixFormat fmt;
            fmt.glformat = GL_LUMINANCE;
            fmt.gltype = GL_UNSIGNED_SHORT;
            fmt.scalable_internal_format = GL_LUMINANCE16;

            img_view[cam_id]->SetImage(
                img_vec[cam_id].img->ptr, img_vec[cam_id].img->w,
                img_vec[cam_id].img->h, img_vec[cam_id].img->pitch, fmt);
          } else {
            img_view[cam_id]->Clear();
          }
        drawPlots();
      }

      if (show_accel.GuiChanged() || show_gyro.GuiChanged() ||
          show_data.GuiChanged() || show_spline.GuiChanged() ||
          show_pos.GuiChanged() || show_rot_error.GuiChanged() ||
          show_mocap.GuiChanged() || show_mocap_rot_error.GuiChanged() ||
          show_mocap_rot_vel.GuiChanged()) {
        drawPlots();
      }
    }

    if (opt_until_convg) {
      bool converged = optimizeWithParam(true);
      if (converged) opt_until_convg = false;
    }

    pangolin::FinishFrame();
  }
}

int CamImuCalib::runHeadless(int max_iterations, bool save_mocap) {
  if (show_gui) {
    std::cerr << "runHeadless requires construction with show_gui=false."
              << std::endl;
    return 1;
  }

  loadDataset();

  if (!calib_opt || !calib_opt->calibInitialized()) {
    std::cerr << "No camera calibration found in the result path. Run "
                 "basalt_calibrate_cam first."
              << std::endl;
    return 1;
  }

  if (!hasCorners()) {
    detectCorners();
  }
  if (!hasCorners()) {
    std::cerr << "No corners detected. Check the dataset and the Aprilgrid "
                 "configuration."
              << std::endl;
    return 1;
  }

  initCamPoses();
  if (calib_init_poses.empty()) {
    std::cerr << "Failed to initialize camera poses." << std::endl;
    return 1;
  }

  initCamImuTransform();
  initOptimization();

  bool converged = false;
  int iteration = 0;
  while (!converged && iteration < max_iterations) {
    converged = optimizeWithParam(true);
    iteration++;
  }

  if (save_mocap) {
    initMocap();
    saveMocapCalib();
  }

  saveCalib();

  if (!converged) {
    std::cerr << "Optimization did not converge after " << max_iterations
              << " iterations. Saved the current estimate." << std::endl;
    return 2;
  }
  return 0;
}

void CamImuCalib::computeProjections() {
  reprojected_corners.clear();

  if (!calib_opt || !vio_dataset) return;

  for (size_t j = 0; j < vio_dataset->get_image_timestamps().size(); ++j) {
    int64_t timestamp_ns = vio_dataset->get_image_timestamps()[j];

    int64_t timestamp_corrected_ns =
        timestamp_ns + calib_opt->getCamTimeOffsetNs();

    if (timestamp_corrected_ns < calib_opt->getMinTimeNs() ||
        timestamp_corrected_ns >= calib_opt->getMaxTimeNs())
      continue;

    for (size_t i = 0; i < calib_opt->calib->intrinsics.size(); i++) {
      TimeCamId tcid(timestamp_ns, i);

      ProjectedCornerData rc;

      Sophus::SE3d T_c_w_ = (calib_opt->getT_w_i(timestamp_corrected_ns) *
                             calib_opt->getCamT_i_c(i))
                                .inverse();

      Eigen::Matrix4d T_c_w = T_c_w_.matrix();

      calib_opt->calib->intrinsics[i].project(
          april_grid.aprilgrid_corner_pos_3d, T_c_w, rc.corners_proj,
          rc.corners_proj_success);

      reprojected_corners.emplace(tcid, rc);
    }
  }
}

void CamImuCalib::detectCorners() {
  if (processing_thread) {
    processing_thread->join();
    processing_thread.reset();
  }

  processing_thread.reset(new std::thread([this]() {
    std::cout << "Started detecting corners" << std::endl;

    CalibHelper::detectCorners(this->vio_dataset, this->april_grid,
                               this->calib_corners,
                               this->calib_corners_rejected);

    std::string path =
        cache_path + cache_dataset_name + "_detected_corners.cereal";
    std::ofstream os(path, std::ios::binary);
    cereal::BinaryOutputArchive archive(os);

    archive(this->calib_corners);
    archive(this->calib_corners_rejected);

    std::cout << "Done detecting corners. Saved them here: " << path
              << std::endl;
  }));

  if (!show_gui) {
    processing_thread->join();
    processing_thread.reset();
  }
}

void CamImuCalib::initCamPoses() {
  if (!cornersComplete()) return;

  if (!calib_opt || !calib_opt->calibInitialized()) {
    std::cerr << "No camera intrinsics. Calibrate camera using "
                 "basalt_calibrate first!"
              << std::endl;
    return;
  }

  if (processing_thread) {
    processing_thread->join();
    processing_thread.reset();
  }
  {
    std::cout << "Started initial camera pose computation " << std::endl;

    CalibHelper::initCamPoses(calib_opt->calib,
                              april_grid.aprilgrid_corner_pos_3d,
                              this->calib_corners, this->calib_init_poses);

    std::string path = cache_path + cache_dataset_name + "_init_poses.cereal";
    std::ofstream os(path, std::ios::binary);
    cereal::BinaryOutputArchive archive(os);

    archive(this->calib_init_poses);

    std::cout << "Done initial camera pose computation. Saved them here: "
              << path << std::endl;
  };
}

void CamImuCalib::initCamImuTransform() {
  if (!calib_opt || !calib_opt->calibInitialized()) {
    std::cerr << "No camera intrinsics. Calibrate camera using "
                 "basalt_calibrate first!"
              << std::endl;
    return;
  }

  if (calib_init_poses.empty()) {
    std::cerr << "Initialize camera poses first!" << std::endl;
    return;
  }

  std::vector<int64_t> timestamps_cam;
  Eigen::aligned_vector<Eigen::Vector3d> rot_vel_cam;
  Eigen::aligned_vector<Eigen::Vector3d> rot_vel_imu;

  Sophus::SO3d R_i_c0_init = calib_opt->getCamT_i_c(0).so3();

  for (size_t i = 1; i < vio_dataset->get_image_timestamps().size(); i++) {
    int64_t timestamp0_ns = vio_dataset->get_image_timestamps()[i - 1];
    int64_t timestamp1_ns = vio_dataset->get_image_timestamps()[i];

    TimeCamId tcid0(timestamp0_ns, 0);
    TimeCamId tcid1(timestamp1_ns, 0);

    if (calib_init_poses.find(tcid0) == calib_init_poses.end()) continue;
    if (calib_init_poses.find(tcid1) == calib_init_poses.end()) continue;

    Sophus::SE3d T_a_c0 = calib_init_poses.at(tcid0).T_a_c;
    Sophus::SE3d T_a_c1 = calib_init_poses.at(tcid1).T_a_c;

    double dt = (timestamp1_ns - timestamp0_ns) * 1e-9;

    Eigen::Vector3d rot_vel_c0 =
        R_i_c0_init * (T_a_c0.so3().inverse() * T_a_c1.so3()).log() / dt;

    timestamps_cam.push_back(timestamp0_ns);
    rot_vel_cam.push_back(rot_vel_c0);
  }

  for (size_t j = 0; j < timestamps_cam.size(); j++) {
    int idx = -1;
    int64_t min_dist = std::numeric_limits<int64_t>::max();

    for (size_t i = 1; i < vio_dataset->get_gyro_data().size(); i++) {
      int64_t dist =
          vio_dataset->get_gyro_data()[i].timestamp_ns - timestamps_cam[j];
      if (std::abs(dist) < min_dist) {
        min_dist = std::abs(dist);
        idx = i;
      }
    }

    rot_vel_imu.push_back(vio_dataset->get_gyro_data()[idx].data);
  }

  BASALT_ASSERT_STREAM(rot_vel_cam.size() == rot_vel_imu.size(),
                       "rot_vel_cam.size() " << rot_vel_cam.size()
                                             << " rot_vel_imu.size() "
                                             << rot_vel_imu.size());

  //  R_i_c * rot_vel_cam = rot_vel_imu
  //  R_i_c * rot_vel_cam * rot_vel_cam.T = rot_vel_imu * rot_vel_cam.T
  //  R_i_c  = rot_vel_imu * rot_vel_cam.T * (rot_vel_cam * rot_vel_cam.T)^-1;

  Eigen::Matrix<double, 3, Eigen::Dynamic> rot_vel_cam_m(3, rot_vel_cam.size()),
      rot_vel_imu_m(3, rot_vel_imu.size());

  for (size_t i = 0; i < rot_vel_cam.size(); i++) {
    rot_vel_cam_m.col(i) = rot_vel_cam[i];
    rot_vel_imu_m.col(i) = rot_vel_imu[i];
  }

  Eigen::Matrix3d R_i_c0 =
      rot_vel_imu_m * rot_vel_cam_m.transpose() *
      (rot_vel_cam_m * rot_vel_cam_m.transpose()).inverse();

  // std::cout << "raw R_i_c0\n" << R_i_c0 << std::endl;

  Eigen::AngleAxisd aa(R_i_c0);  // RotationMatrix to AxisAngle
  R_i_c0 = aa.toRotationMatrix();

  // std::cout << "R_i_c0\n" << R_i_c0 << std::endl;

  Sophus::SE3d T_i_c0(R_i_c0, Eigen::Vector3d::Zero());

  std::cout << "T_i_c0\n" << T_i_c0.matrix() << std::endl;

  for (size_t i = 0; i < vio_dataset->get_num_cams(); i++) {
    calib_opt->getCamT_i_c(i) = T_i_c0 * calib_opt->getCamT_i_c(i);
  }

  std::cout << "Done Camera-IMU extrinsics initialization:" << std::endl;
  for (size_t j = 0; j < vio_dataset->get_num_cams(); j++) {
    std::cout << "T_i_c" << j << ":\n"
              << calib_opt->getCamT_i_c(j).matrix() << std::endl;
  }
}

void CamImuCalib::initOptimization() {
  if (!cornersComplete()) return;

  if (!calib_opt || !calib_opt->calibInitialized()) {
    std::cerr << "No camera intrinsics. Calibrate camera using "
                 "basalt_calibrate first!"
              << std::endl;
    return;
  }

  calib_opt->setAprilgridCorners3d(april_grid.aprilgrid_corner_pos_3d);

  for (size_t i = 0; i < vio_dataset->get_accel_data().size(); i++) {
    const basalt::AccelData &ad = vio_dataset->get_accel_data()[i];
    const basalt::GyroData &gd = vio_dataset->get_gyro_data()[i];

    calib_opt->addAccelMeasurement(ad.timestamp_ns, ad.data);
    calib_opt->addGyroMeasurement(gd.timestamp_ns, gd.data);
  }

  // Skip cached corners outside the current image selection; adding them
  // would expand the spline over the whole recording.
  const std::unordered_set<int64_t> selected_timestamps(
      vio_dataset->get_image_timestamps().begin(),
      vio_dataset->get_image_timestamps().end());

  for (const auto &kv : calib_corners) {
    if (selected_timestamps.count(kv.first.frame_id) == 0) continue;
    if (kv.second.corner_ids.size() >= MIN_CORNERS)
      calib_opt->addAprilgridMeasurement(kv.first.frame_id, kv.first.cam_id,
                                         kv.second.corners,
                                         kv.second.corner_ids);
  }

  for (size_t i = 0; i < vio_dataset->get_gt_timestamps().size(); i++) {
    calib_opt->addMocapMeasurement(vio_dataset->get_gt_timestamps()[i],
                                   vio_dataset->get_gt_pose_data()[i]);
  }

  bool g_initialized = false;
  Eigen::Vector3d g_a_init;

  for (size_t j = 0; j < vio_dataset->get_image_timestamps().size(); ++j) {
    int64_t timestamp_ns = vio_dataset->get_image_timestamps()[j];

    TimeCamId tcid(timestamp_ns, 0);
    const auto cp_it = calib_init_poses.find(tcid);

    if (cp_it != calib_init_poses.end()) {
      Sophus::SE3d T_a_i =
          cp_it->second.T_a_c * calib_opt->getCamT_i_c(0).inverse();

      calib_opt->addPoseMeasurement(timestamp_ns, T_a_i);

      if (!g_initialized) {
        for (size_t i = 0;
             i < vio_dataset->get_accel_data().size() && !g_initialized; i++) {
          const basalt::AccelData &ad = vio_dataset->get_accel_data()[i];
          if (std::abs(ad.timestamp_ns - timestamp_ns) < 3000000) {
            g_a_init = T_a_i.so3() * ad.data;
            g_initialized = true;
            std::cout << "g_a initialized with " << g_a_init.transpose()
                      << std::endl;
          }
        }
      }
    }
  }

  if (calib_opt->calib->imu_update_rate == 0)
    calib_opt->calib->imu_update_rate =
        calculateRate(vio_dataset->get_gyro_data());

  std::cout << "IMU freq: " << calib_opt->calib->imu_update_rate << std::endl;

  calib_opt->setG(g_a_init);
  calib_opt->init();
  computeProjections();
  recomputeDataLog();

  std::cout << "Initialized optimization." << std::endl;
}

void CamImuCalib::initMocap() {
  if (!calib_opt || !calib_opt->calibInitialized()) {
    std::cerr << "Initalize optimization first!" << std::endl;
    return;
  }

  if (vio_dataset->get_gt_timestamps().empty()) {
    std::cerr << "The dataset contains no Mocap data!" << std::endl;
    return;
  }

  {
    std::vector<int64_t> timestamps_cam;
    Eigen::aligned_vector<Eigen::Vector3d> rot_vel_mocap;
    Eigen::aligned_vector<Eigen::Vector3d> rot_vel_imu;

    Sophus::SO3d R_i_mark_init = calib_opt->mocap_calib->T_i_mark.so3();

    for (size_t i = 1; i < vio_dataset->get_gt_timestamps().size(); i++) {
      int64_t timestamp0_ns = vio_dataset->get_gt_timestamps()[i - 1];
      int64_t timestamp1_ns = vio_dataset->get_gt_timestamps()[i];

      Sophus::SE3d T_a_mark0 = vio_dataset->get_gt_pose_data()[i - 1];
      Sophus::SE3d T_a_mark1 = vio_dataset->get_gt_pose_data()[i];

      double dt = (timestamp1_ns - timestamp0_ns) * 1e-9;

      Eigen::Vector3d rot_vel_c0 =
          R_i_mark_init * (T_a_mark0.so3().inverse() * T_a_mark1.so3()).log() /
          dt;

      timestamps_cam.push_back(timestamp0_ns);
      rot_vel_mocap.push_back(rot_vel_c0);
    }

    for (size_t j = 0; j < timestamps_cam.size(); j++) {
      int idx = -1;
      int64_t min_dist = std::numeric_limits<int64_t>::max();

      for (size_t i = 1; i < vio_dataset->get_gyro_data().size(); i++) {
        int64_t dist =
            vio_dataset->get_gyro_data()[i].timestamp_ns - timestamps_cam[j];
        if (std::abs(dist) < min_dist) {
          min_dist = std::abs(dist);
          idx = i;
        }
      }

      rot_vel_imu.push_back(vio_dataset->get_gyro_data()[idx].data);
    }

    BASALT_ASSERT_STREAM(rot_vel_mocap.size() == rot_vel_imu.size(),
                         "rot_vel_cam.size() " << rot_vel_mocap.size()
                                               << " rot_vel_imu.size() "
                                               << rot_vel_imu.size());

    //  R_i_c * rot_vel_mocap = rot_vel_imu
    //  R_i_c * rot_vel_mocap * rot_vel_mocap.T = rot_vel_imu * rot_vel_mocap.T
    //  R_i_c  = rot_vel_imu * rot_vel_mocap.T * (rot_vel_mocap *
    //  rot_vel_mocap.T)^-1;

    Eigen::Matrix<double, 3, Eigen::Dynamic> rot_vel_mocap_m(
        3, rot_vel_mocap.size()),
        rot_vel_imu_m(3, rot_vel_imu.size());

    for (size_t i = 0; i < rot_vel_mocap.size(); i++) {
      rot_vel_mocap_m.col(i) = rot_vel_mocap[i];
      rot_vel_imu_m.col(i) = rot_vel_imu[i];
    }

    Eigen::Matrix3d R_i_mark =
        rot_vel_imu_m * rot_vel_mocap_m.transpose() *
        (rot_vel_mocap_m * rot_vel_mocap_m.transpose()).inverse();

    // std::cout << "raw R_i_c0\n" << R_i_c0 << std::endl;

    Eigen::AngleAxisd aa(R_i_mark);  // RotationMatrix to AxisAngle
    R_i_mark = aa.toRotationMatrix();

    Sophus::SE3d T_i_mark_new(R_i_mark, Eigen::Vector3d::Zero());
    calib_opt->mocap_calib->T_i_mark =
        T_i_mark_new * calib_opt->mocap_calib->T_i_mark;

    std::cout << "Initialized T_i_mark:\n"
              << calib_opt->mocap_calib->T_i_mark.matrix() << std::endl;
  }

  // Initialize T_w_moc;
  Sophus::SE3d T_w_moc;

  // TODO: check for failure cases..
  for (size_t i = vio_dataset->get_gt_timestamps().size() / 2;
       i < vio_dataset->get_gt_timestamps().size(); i++) {
    int64_t timestamp_ns = vio_dataset->get_gt_timestamps()[i];
    T_w_moc = calib_opt->getT_w_i(timestamp_ns) *
              calib_opt->mocap_calib->T_i_mark *
              vio_dataset->get_gt_pose_data()[i].inverse();

    std::cout << "Initialized T_w_moc:\n" << T_w_moc.matrix() << std::endl;
    break;
  }

  calib_opt->setT_w_moc(T_w_moc);
  calib_opt->mocap_initialized = true;

  recomputeDataLog();
}

void CamImuCalib::loadDataset() {
  basalt::DatasetIoInterfacePtr dataset_io =
      basalt::DatasetIoFactory::getDatasetIo(dataset_type);

  dataset_io->read(dataset_path);

  vio_dataset = dataset_io->get_data();
  setNumCameras(vio_dataset->get_num_cams());

  if (skip_images > 1 || start_image > 0 || end_image != 0) {
    const size_t num_images = vio_dataset->get_image_timestamps().size();
    const size_t first_idx = start_image > 0 ? start_image : 0;
    const size_t last_idx =
        end_image > 0
            ? std::min(size_t(end_image), num_images)
            : (num_images >= size_t(-end_image) ? num_images + end_image : 0);

    std::vector<int64_t> new_image_timestamps;
    new_image_timestamps.reserve(num_images);
    for (size_t i = first_idx; i < last_idx; ++i) {
      if (i % skip_images == 0) {
        new_image_timestamps.push_back(vio_dataset->get_image_timestamps()[i]);
      }
    }

    if (new_image_timestamps.empty()) {
      std::cerr << "No images left after applying the start/end/skip image "
                   "options. Aborting."
                << std::endl;
      std::abort();
    }

    vio_dataset->get_image_timestamps() = new_image_timestamps;

    // Keep IMU data slightly beyond the selected image interval, so that the
    // spline stays constrained at the first and last frame even with a
    // moderate camera-IMU time offset.
    constexpr int64_t TRIM_MARGIN_NS = 100000000;  // 100 ms
    const int64_t keep_from = new_image_timestamps.front() - TRIM_MARGIN_NS;
    const int64_t keep_to = new_image_timestamps.back() + TRIM_MARGIN_NS;

    const auto trim_imu_data = [&](auto &data, const char *name) {
      if (data.empty()) return;
      auto first = std::lower_bound(
          data.begin(), data.end(), keep_from,
          [](const auto &meas, int64_t t) { return meas.timestamp_ns < t; });
      if (first != data.begin()) --first;  // bracketing sample before
      auto last = std::upper_bound(
          first, data.end(), keep_to,
          [](int64_t t, const auto &meas) { return t < meas.timestamp_ns; });
      if (last != data.end()) ++last;  // bracketing sample after
      std::cout << "Original " << name << " data size: " << data.size()
                << " new size: " << std::distance(first, last)
                << " trimmed elements at the beginning: "
                << std::distance(data.begin(), first)
                << " trimmed elements at the end: "
                << std::distance(last, data.end()) << std::endl;
      data.erase(last, data.end());
      data.erase(data.begin(), first);
    };

    trim_imu_data(vio_dataset->get_accel_data(), "accel");
    trim_imu_data(vio_dataset->get_gyro_data(), "gyro");
  }

  // load detected corners if they exist
  {
    std::string path =
        cache_path + cache_dataset_name + "_detected_corners.cereal";

    std::ifstream is(path, std::ios::binary);

    if (is.good()) {
      cereal::BinaryInputArchive archive(is);

      calib_corners.clear();
      calib_corners_rejected.clear();
      archive(calib_corners);
      archive(calib_corners_rejected);

      std::cout << "Loaded detected corners from: " << path << std::endl;

      const size_t num_missing =
          CalibHelper::getMissingCornerFrames(
              vio_dataset->get_image_timestamps(), calib_corners)
              .size();
      if (num_missing > 0) {
        std::cerr << "Warning: " << num_missing << " of "
                  << vio_dataset->get_image_timestamps().size()
                  << " selected images have no corner data in the cache "
                     "(created with different start/end/skip image options?). "
                     "Press detect_corners to detect the missing frames."
                  << std::endl;
      }
    } else {
      std::cout << "No pre-processed detected corners found" << std::endl;
    }
  }

  // load initial poses if they exist
  {
    std::string path = cache_path + cache_dataset_name + "_init_poses.cereal";

    std::ifstream is(path, std::ios::binary);

    if (is.good()) {
      cereal::BinaryInputArchive archive(is);

      calib_init_poses.clear();
      archive(calib_init_poses);

      std::cout << "Loaded initial poses from: " << path << std::endl;
    } else {
      std::cout << "No pre-processed initial poses found" << std::endl;
    }
  }

  // load calibration if exist
  {
    if (!calib_opt) calib_opt.reset(new SplineOptimization<5, double>);

    calib_opt->loadCalib(cache_path);

    if (calib_opt->calibInitialized()) {
      if (calib_opt->calib->imu_update_rate == 0) {
        calib_opt->calib->accel_noise_std.setConstant(imu_noise[0]);
        calib_opt->calib->gyro_noise_std.setConstant(imu_noise[1]);
        calib_opt->calib->accel_bias_std.setConstant(imu_noise[2]);
        calib_opt->calib->gyro_bias_std.setConstant(imu_noise[3]);

        std::cout << "No IMU calibration found, using defaults" << std::endl;
      } else {
        std::cout << "Found IMU calibration" << std::endl;
      }
    }
  }
  calib_opt->resetMocapCalib();

  reprojected_corners.clear();

  if (show_gui) {
    show_frame = 0;

    show_frame.Meta().range[1] = vio_dataset->get_image_timestamps().size() - 1;
    show_frame.Meta().gui_changed = true;

    plotter->ClearSeries();
    recomputeDataLog();
    drawPlots();
  }
}

void CamImuCalib::optimize() { optimizeWithParam(true); }

bool CamImuCalib::optimizeWithParam(bool print_info,
                                    std::map<std::string, double> *stats) {
  if (!calib_opt || !calib_opt->calibInitialized()) {
    std::cerr << "Initalize optimization first!" << std::endl;
    return true;
  }

  bool converged = true;

  if (calib_opt) {
    // calib_opt->compute_projections();
    double error;
    double reprojection_error;
    int num_points;

    auto start = std::chrono::high_resolution_clock::now();

    converged = calib_opt->optimize(opt_intr, opt_poses, opt_corners,
                                    opt_cam_time_offset, opt_imu_scale,
                                    opt_mocap, huber_thresh, stop_thresh, error,
                                    num_points, reprojection_error);

    auto finish = std::chrono::high_resolution_clock::now();

    if (stats) {
      stats->clear();

      stats->emplace("energy_error", error);
      stats->emplace("num_points", num_points);
      stats->emplace("mean_energy_error", error / num_points);
      stats->emplace("reprojection_error", reprojection_error);
      stats->emplace("mean_reprojection_error",
                     reprojection_error / num_points);
    }

    if (print_info) {
      std::cout << "==================================" << std::endl;

      for (size_t i = 0; i < vio_dataset->get_num_cams(); i++) {
        std::cout << "intrinsics " << i << ": "
                  << calib_opt->getIntrinsics(i).transpose() << std::endl;
        std::cout << "T_i_c" << i << ":\n"
                  << calib_opt->getCamT_i_c(i).matrix() << std::endl;
      }

      std::cout << "T_w_moc:\n"
                << calib_opt->getT_w_moc().matrix() << std::endl;

      std::cout << "T_mark_i:\n"
                << calib_opt->getT_mark_i().matrix() << std::endl;

      std::cout << "cam_time_offset_ns: " << calib_opt->getCamTimeOffsetNs()
                << std::endl;

      std::cout << "mocap_time_offset_ns: " << calib_opt->getMocapTimeOffsetNs()
                << std::endl;
      {
        Eigen::Vector3d accel_bias;
        Eigen::Matrix3d accel_scale;
        calib_opt->getAccelBias().getBiasAndScale(accel_bias, accel_scale);

        std::cout << "accel_bias: " << accel_bias.transpose()
                  << "\naccel_scale:\n"
                  << Eigen::Matrix3d::Identity() + accel_scale << std::endl;

        Eigen::Vector3d gyro_bias;
        Eigen::Matrix3d gyro_scale;
        calib_opt->getGyroBias().getBiasAndScale(gyro_bias, gyro_scale);

        std::cout << "gyro_bias: " << gyro_bias.transpose() << "\ngyro_scale:\n"
                  << Eigen::Matrix3d::Identity() + gyro_scale << std::endl;
      }

      std::cout << " g " << calib_opt->getG().transpose()
                << " norm: " << calib_opt->getG().norm() << " g_mocap: "
                << (calib_opt->getT_w_moc().inverse().so3() * calib_opt->getG())
                       .transpose()
                << std::endl;

      std::cout << "Current error: " << error << " num_points " << num_points
                << " mean_error " << error / num_points
                << " reprojection_error " << reprojection_error
                << " mean reprojection " << reprojection_error / num_points
                << " opt_time "
                << std::chrono::duration_cast<std::chrono::milliseconds>(
                       finish - start)
                       .count()
                << "ms." << std::endl;

      if (converged) std::cout << "Optimization Converged !!" << std::endl;

      std::cout << "==================================" << std::endl;
    }

    // calib_opt->compute_error(error, num_points);
    // std::cerr << "after opt error: " << error << " num_points " <<
    // num_points << std::endl;

    if (show_gui) {
      computeProjections();
      recomputeDataLog();
      drawPlots();
    }
  }

  return converged;
}

void CamImuCalib::saveCalib() {
  if (calib_opt) {
    calib_opt->saveCalib(cache_path);

    std::cout << "Saved calibration in " << cache_path << "calibration.json"
              << std::endl;
  }
}

void CamImuCalib::saveMocapCalib() {
  if (calib_opt) {
    calib_opt->saveMocapCalib(cache_path,
                              vio_dataset->get_mocap_to_imu_offset_ns());

    std::cout << "Saved Mocap calibration in " << cache_path
              << "mocap_calibration.json" << std::endl;
  }
}

void CamImuCalib::drawImageOverlay(pangolin::View &v, size_t cam_id) {
  UNUSED(v);

  size_t frame_id = show_frame;

  if (vio_dataset && frame_id < vio_dataset->get_image_timestamps().size()) {
    int64_t timestamp_ns = vio_dataset->get_image_timestamps()[frame_id];
    TimeCamId tcid(timestamp_ns, cam_id);

    if (show_corners) {
      glLineWidth(1.0);
      glColor3f(1.0, 0.0, 0.0);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

      if (calib_corners.find(tcid) != calib_corners.end()) {
        const CalibCornerData &cr = calib_corners.at(tcid);
        const CalibCornerData &cr_rej = calib_corners_rejected.at(tcid);

        for (size_t i = 0; i < cr.corners.size(); i++) {
          // the radius is the threshold used for maximum displacement. The
          // search region is slightly larger.
          const float radius = static_cast<float>(cr.radii[i]);
          const Eigen::Vector2f c = cr.corners[i].cast<float>();
          pangolin::glDrawCirclePerimeter(c[0], c[1], radius);

          if (show_ids)
            pangolin::GlFont::I().Text("%d", cr.corner_ids[i]).Draw(c[0], c[1]);
        }

        pangolin::GlFont::I()
            .Text("Detected %d corners (%d rejected)", cr.corners.size(),
                  cr_rej.corners.size())
            .Draw(5, 50);

        if (show_corners_rejected) {
          glColor3f(1.0, 0.5, 0.0);

          for (size_t i = 0; i < cr_rej.corners.size(); i++) {
            // the radius is the threshold used for maximum displacement. The
            // search region is slightly larger.
            const float radius = static_cast<float>(cr_rej.radii[i]);
            const Eigen::Vector2f c = cr_rej.corners[i].cast<float>();
            pangolin::glDrawCirclePerimeter(c[0], c[1], radius);

            if (show_ids)
              pangolin::GlFont::I()
                  .Text("%d", cr_rej.corner_ids[i])
                  .Draw(c[0], c[1]);
          }
        }

      } else {
        glLineWidth(1.0);

        pangolin::GlFont::I().Text("Corners not processed").Draw(5, 50);
      }
    }

    if (show_init_reproj) {
      glLineWidth(1.0);
      glColor3f(1.0, 1.0, 0.0);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

      if (calib_init_poses.find(tcid) != calib_init_poses.end()) {
        const CalibInitPoseData &cr = calib_init_poses.at(tcid);

        for (size_t i = 0; i < cr.reprojected_corners.size(); i++) {
          Eigen::Vector2d c = cr.reprojected_corners[i];
          pangolin::glDrawCirclePerimeter(c[0], c[1], 3.0);

          if (show_ids) pangolin::GlFont::I().Text("%d", i).Draw(c[0], c[1]);
        }

        pangolin::GlFont::I()
            .Text("Initial pose with %d inliers", cr.num_inliers)
            .Draw(5, 100);

      } else {
        pangolin::GlFont::I().Text("Initial pose not processed").Draw(5, 100);
      }
    }

    if (show_opt) {
      glLineWidth(1.0);
      glColor3f(1.0, 0.0, 1.0);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

      if (reprojected_corners.find(tcid) != reprojected_corners.end()) {
        if (calib_corners.count(tcid) > 0 &&
            calib_corners.at(tcid).corner_ids.size() >= MIN_CORNERS) {
          const auto &rc = reprojected_corners.at(tcid);

          for (size_t i = 0; i < rc.corners_proj.size(); i++) {
            if (!rc.corners_proj_success[i]) continue;

            Eigen::Vector2d c = rc.corners_proj[i];
            pangolin::glDrawCirclePerimeter(c[0], c[1], 3.0);

            if (show_ids) pangolin::GlFont::I().Text("%d", i).Draw(c[0], c[1]);
          }
        } else {
          pangolin::GlFont::I().Text("Too few corners detected.").Draw(5, 150);
        }
      }
    }
  }
}

void CamImuCalib::recomputeDataLog() {
  imu_data_log.Clear();
  pose_data_log.Clear();
  mocap_data_log.Clear();

  if (!vio_dataset || vio_dataset->get_accel_data().empty()) return;

  double min_time = vio_dataset->get_accel_data().front().timestamp_ns * 1e-9;
  max_time = vio_dataset->get_accel_data().back().timestamp_ns * 1e-9 - min_time;

  for (size_t i = 0; i < vio_dataset->get_accel_data().size(); i++) {
    const basalt::AccelData &ad = vio_dataset->get_accel_data()[i];
    const basalt::GyroData &gd = vio_dataset->get_gyro_data()[i];

    Eigen::Vector3d a_sp(0, 0, 0), g_sp(0, 0, 0);

    if (calib_opt && calib_opt->calibInitialized() &&
        calib_opt->initialized()) {
      Sophus::SE3d pose_sp = calib_opt->getT_w_i(ad.timestamp_ns);
      Eigen::Vector3d a_sp_w = calib_opt->getTransAccelWorld(ad.timestamp_ns);

      a_sp = calib_opt->getAccelBias().invertCalibration(
          pose_sp.so3().inverse() * (a_sp_w + calib_opt->getG()));

      g_sp = calib_opt->getGyroBias().invertCalibration(
          calib_opt->getRotVelBody(ad.timestamp_ns));
    }

    std::vector<float> vals;
    double t = ad.timestamp_ns * 1e-9 - min_time;
    vals.push_back(t);

    for (int k = 0; k < 3; k++) vals.push_back(ad.data[k]);
    for (int k = 0; k < 3; k++) vals.push_back(a_sp[k]);
    for (int k = 0; k < 3; k++) vals.push_back(gd.data[k]);
    for (int k = 0; k < 3; k++) vals.push_back(g_sp[k]);

    imu_data_log.Log(vals);
  }

  for (size_t i = 0; i < vio_dataset->get_image_timestamps().size(); i++) {
    int64_t timestamp_ns = vio_dataset->get_image_timestamps()[i];

    TimeCamId tcid(timestamp_ns, 0);
    const auto &it = calib_init_poses.find(tcid);

    double t = timestamp_ns * 1e-9 - min_time;

    Sophus::SE3d pose_sp, pose_meas;
    if (calib_opt && calib_opt->initialized())
      pose_sp = calib_opt->getT_w_i(timestamp_ns);

    if (it != calib_init_poses.end() && calib_opt &&
        calib_opt->calibInitialized())
      pose_meas = it->second.T_a_c * calib_opt->getCamT_i_c(0).inverse();

    Eigen::Vector3d p_sp = pose_sp.translation();
    Eigen::Vector3d p_meas = pose_meas.translation();

    double angle =
        pose_sp.unit_quaternion().angularDistance(pose_meas.unit_quaternion()) *
        180 / M_PI;

    pose_data_log.Log(t, p_meas[0], p_meas[1], p_meas[2], p_sp[0], p_sp[1],
                      p_sp[2], angle);
  }

  for (size_t i = 0; i < vio_dataset->get_gt_timestamps().size(); i++) {
    int64_t timestamp_ns = vio_dataset->get_gt_timestamps()[i];

    if (calib_opt && calib_opt->calibInitialized())
      timestamp_ns += calib_opt->getMocapTimeOffsetNs();

    double t = timestamp_ns * 1e-9 - min_time;

    Sophus::SE3d pose_sp, pose_mocap;
    if (calib_opt && calib_opt->calibInitialized()) {
      if (timestamp_ns < calib_opt->getMinTimeNs() ||
          timestamp_ns > calib_opt->getMaxTimeNs())
        continue;

      pose_sp = calib_opt->getT_w_i(timestamp_ns);
      pose_mocap = calib_opt->getT_w_moc() *
                   vio_dataset->get_gt_pose_data()[i] *
                   calib_opt->getT_mark_i();
    }

    Eigen::Vector3d p_sp = pose_sp.translation();
    Eigen::Vector3d p_mocap = pose_mocap.translation();

    double angle = pose_sp.unit_quaternion().angularDistance(
                       pose_mocap.unit_quaternion()) *
                   180 / M_PI;

    Eigen::Vector3d rot_vel(0, 0, 0);
    if (i > 0) {
      double dt = (vio_dataset->get_gt_timestamps()[i] -
                   vio_dataset->get_gt_timestamps()[i - 1]) *
                  1e-9;

      rot_vel = (vio_dataset->get_gt_pose_data()[i - 1].so3().inverse() *
                 vio_dataset->get_gt_pose_data()[i].so3())
                    .log() /
                dt;
    }

    std::vector<double> valsd = {t,          p_mocap[0], p_mocap[1], p_mocap[2],
                                 p_sp[0],    p_sp[1],    p_sp[2],    angle,
                                 rot_vel[0], rot_vel[1], rot_vel[2]};

    std::vector<float> vals(valsd.begin(), valsd.end());

    mocap_data_log.Log(vals);
  }
}

void CamImuCalib::drawPlots() {
  plotter->ClearSeries();
  plotter->ClearMarkers();

  float plotter_min = -1.0f, plotter_max = 1.0f;

  if (show_accel) {
    if (show_data) {
      plotter->AddSeries("$0", "$1", pangolin::DrawingModeDashed,
                         pangolin::Colour::Red(), "a x");
      plotter->AddSeries("$0", "$2", pangolin::DrawingModeDashed,
                         pangolin::Colour::Green(), "a y");
      plotter->AddSeries("$0", "$3", pangolin::DrawingModeDashed,
                         pangolin::Colour::Blue(), "a z");
    }

    if (show_spline) {
      plotter->AddSeries("$0", "$4", pangolin::DrawingModeLine,
                         pangolin::Colour::Red(), "a x Spline");
      plotter->AddSeries("$0", "$5", pangolin::DrawingModeLine,
                         pangolin::Colour::Green(), "a y Spline");
      plotter->AddSeries("$0", "$6", pangolin::DrawingModeLine,
                         pangolin::Colour::Blue(), "a z Spline");
    }

    plotter_min = std::min(plotter_min, -15.0f);
    plotter_max = std::max(plotter_max, 15.0f);
  }

  if (show_gyro) {
    if (show_data) {
      plotter->AddSeries("$0", "$7", pangolin::DrawingModeDashed,
                         pangolin::Colour::Red(), "g x");
      plotter->AddSeries("$0", "$8", pangolin::DrawingModeDashed,
                         pangolin::Colour::Green(), "g y");
      plotter->AddSeries("$0", "$9", pangolin::DrawingModeDashed,
                         pangolin::Colour::Blue(), "g z");
    }

    if (show_spline) {
      plotter->AddSeries("$0", "$10", pangolin::DrawingModeLine,
                         pangolin::Colour::Red(), "g x Spline");
      plotter->AddSeries("$0", "$11", pangolin::DrawingModeLine,
                         pangolin::Colour::Green(), "g y Spline");
      plotter->AddSeries("$0", "$12", pangolin::DrawingModeLine,
                         pangolin::Colour::Blue(), "g z Spline");
    }

    plotter_min = std::min(plotter_min, -5.0f);
    plotter_max = std::max(plotter_max, 5.0f);
  }

  if (show_pos) {
    if (show_data) {
      plotter->AddSeries("$0", "$1", pangolin::DrawingModeDashed,
                         pangolin::Colour::Red(), "p x", &pose_data_log);
      plotter->AddSeries("$0", "$2", pangolin::DrawingModeDashed,
                         pangolin::Colour::Green(), "p y", &pose_data_log);
      plotter->AddSeries("$0", "$3", pangolin::DrawingModeDashed,
                         pangolin::Colour::Blue(), "p z", &pose_data_log);
    }

    if (show_spline) {
      plotter->AddSeries("$0", "$4", pangolin::DrawingModeLine,
                         pangolin::Colour::Red(), "p x Spline", &pose_data_log);
      plotter->AddSeries("$0", "$5", pangolin::DrawingModeLine,
                         pangolin::Colour::Green(), "p y Spline",
                         &pose_data_log);
      plotter->AddSeries("$0", "$6", pangolin::DrawingModeLine,
                         pangolin::Colour::Blue(), "p z Spline",
                         &pose_data_log);
    }

    plotter_min = std::min(plotter_min, -2.0f);
    plotter_max = std::max(plotter_max, 2.0f);
  }

  if (show_rot_error) {
    plotter->AddSeries("$0", "$7", pangolin::DrawingModeLine,
                       pangolin::Colour::White(), "rot error", &pose_data_log);
  }

  if (show_mocap) {
    if (show_data) {
      plotter->AddSeries("$0", "$1", pangolin::DrawingModeDashed,
                         pangolin::Colour::Red(), "p x", &mocap_data_log);
      plotter->AddSeries("$0", "$2", pangolin::DrawingModeDashed,
                         pangolin::Colour::Green(), "p y", &mocap_data_log);
      plotter->AddSeries("$0", "$3", pangolin::DrawingModeDashed,
                         pangolin::Colour::Blue(), "p z", &mocap_data_log);
    }

    if (show_spline) {
      plotter->AddSeries("$0", "$4", pangolin::DrawingModeLine,
                         pangolin::Colour::Red(), "p x Spline",
                         &mocap_data_log);
      plotter->AddSeries("$0", "$5", pangolin::DrawingModeLine,
                         pangolin::Colour::Green(), "p y Spline",
                         &mocap_data_log);
      plotter->AddSeries("$0", "$6", pangolin::DrawingModeLine,
                         pangolin::Colour::Blue(), "p z Spline",
                         &mocap_data_log);
    }
  }

  if (show_mocap_rot_error) {
    plotter->AddSeries("$0", "$7", pangolin::DrawingModeLine,
                       pangolin::Colour::White(), "rot error", &mocap_data_log);
  }

  if (show_mocap_rot_vel) {
    plotter->AddSeries("$0", "$8", pangolin::DrawingModeLine,
                       pangolin::Colour(1, 1, 0), "rot vel x", &mocap_data_log);
    plotter->AddSeries("$0", "$9", pangolin::DrawingModeLine,
                       pangolin::Colour(1, 0, 1), "rot vel y", &mocap_data_log);
    plotter->AddSeries("$0", "$10", pangolin::DrawingModeLine,
                       pangolin::Colour(0, 1, 1), "rot vel z", &mocap_data_log);
  }

  plotter->SetView(pangolin::XYRangef(0.0f, max_time, plotter_min, plotter_max));

  size_t frame_id = show_frame;
  double min_time = vio_dataset->get_accel_data().empty()
                        ? vio_dataset->get_image_timestamps()[0] * 1e-9
                        : vio_dataset->get_accel_data()[0].timestamp_ns * 1e-9;

  int64_t timestamp = vio_dataset->get_image_timestamps()[frame_id];
  if (calib_opt && calib_opt->calibInitialized())
    timestamp += calib_opt->getCamTimeOffsetNs();

  double t = timestamp * 1e-9 - min_time;
  plotter->AddMarker(pangolin::Marker::Vertical, t, pangolin::Marker::Equal,
                     pangolin::Colour::White());
}

bool CamImuCalib::hasCorners() const { return !calib_corners.empty(); }

bool CamImuCalib::cornersComplete() const {
  if (calib_corners.empty()) {
    std::cerr << "No corners detected. Press detect_corners to start corner "
                 "detection."
              << std::endl;
    return false;
  }

  const size_t num_missing =
      CalibHelper::getMissingCornerFrames(vio_dataset->get_image_timestamps(),
                                          calib_corners)
          .size();
  if (num_missing > 0) {
    std::cerr << "Corners are missing for " << num_missing << " of "
              << vio_dataset->get_image_timestamps().size()
              << " selected images. Press detect_corners first." << std::endl;
    return false;
  }

  return true;
}

}  // namespace basalt
