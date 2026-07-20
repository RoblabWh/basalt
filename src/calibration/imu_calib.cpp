#include <basalt/calibration/imu_calib.h>

#include <basalt/utils/system_utils.h>

#include <basalt/serialization/headers_serialization.h>

#include <basalt/utils/filesystem.h>

#include <basalt/calibration/allan_variance.h>

#include <cctype>

namespace basalt {

const uint64_t LOG_DATAPOINTS = 10000;

ImuCalib::ImuCalib(const ImuCalibOptions &options)
    : dataset_path(options.dataset_path),
      dataset_type(options.dataset_type),
      sensor_filter{options.sensor_include, options.sensor_exclude,
                    SensorTypes::Imu},
      cache_path(ensure_trailing_slash(options.cache_path)),
      cache_dataset_name(options.cache_dataset_name),
      period_min(options.period_min),
      period_max(options.period_max),
      show_gui(options.show_gui),
      show_data("ui.show_data", true, false, true),
      center_data("ui.center_data", true, false, true),
      show_accel("ui.show_accel", true, false, true),
      show_gyro("ui.show_gyro", true, false, true),
      show_wn("ui.show_wn", true, false, true),
      show_rr("ui.show_rr", true, false, true),
      load_dataset("ui.load_dataset", std::bind(&ImuCalib::loadDataset, this)),
      comp("ui.compute", std::bind(&ImuCalib::compute, this)),
      wn_min("ui.wn_min", options.wn_min, options.period_min,
             options.period_max),
      wn_max("ui.wn_max", options.wn_max, options.period_min,
             options.period_max),
      rr_min("ui.rr_min", options.rr_min, options.period_min,
             options.period_max),
      rr_max("ui.rr_max", options.rr_max, options.period_min,
             options.period_max),
      fit("ui.fit_lines", std::bind(&ImuCalib::fitLines, this)),
      save_calib("ui.save_calib", std::bind(&ImuCalib::saveCalib, this)) {
  if (show_gui) initGui();

  if (!fs::exists(cache_path)) {
    fs::create_directory(cache_path);
  }
}

ImuCalib::~ImuCalib() {}

void ImuCalib::initGui() {
  pangolin::CreateWindowAndBind("Main", 1600, 1000);

  pangolin::View &plot_raw_display = pangolin::CreateDisplay().SetBounds(
      0.66, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0);
  pangolin::View &plot_calib_display = pangolin::CreateDisplay().SetBounds(
      0.0, 0.66, pangolin::Attach::Pix(UI_WIDTH), 1.0);

  pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0,
                                        pangolin::Attach::Pix(UI_WIDTH));

  plotter_raw = new pangolin::Plotter(&imu_raw_log, 0.0, 1000.0, -10.0, 10.0,
                                      0.001f, 0.001f);
  plot_raw_display.AddDisplay(*plotter_raw);
  plotter_calib =
      new pangolin::Plotter(&imu_calib_log, std::log10(period_min) - 0.1,
                            std::log10(period_max) + 0.1, -8.0, 0.0, 0.1, 0.1);
  plot_calib_display.AddDisplay(*plotter_calib);
}

void ImuCalib::renderingLoop() {
  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (vio_dataset) {
      if (show_data.GuiChanged() || center_data.GuiChanged() ||
          show_accel.GuiChanged() || show_gyro.GuiChanged() ||
          show_wn.GuiChanged() || show_rr.GuiChanged()) {
        drawPlots();
      }
    }

    pangolin::FinishFrame();
  }
}

int ImuCalib::runHeadless() {
  if (show_gui) {
    std::cerr << "runHeadless requires construction with show_gui=false."
              << std::endl;
    return 1;
  }

  loadDataset();
  compute();
  fitLines();
  saveCalib();

  return 0;
}

std::string ImuCalib::allanCachePath(bool legacy) const {
  std::string suffix;
  if (!legacy && !imu_name.empty()) {
    std::string sanitized = imu_name;
    for (char &chr : sanitized) {
      if (std::isalnum(static_cast<unsigned char>(chr)) == 0 && chr != '.' &&
          chr != '_' && chr != '-') {
        chr = '_';
      }
    }
    sanitized.erase(0, std::min(sanitized.find_first_not_of('_'),
                                sanitized.size() - 1));
    suffix = "_" + sanitized;
  }
  return cache_path + cache_dataset_name + suffix + "_allan_deviations_tau_" +
         std::to_string(period_min) + ".cereal";
}

void ImuCalib::loadDataset() {
  basalt::DatasetIoInterfacePtr dataset_io =
      basalt::DatasetIoFactory::getDatasetIo(dataset_type, sensor_filter);

  dataset_io->read(dataset_path);

  vio_dataset = dataset_io->get_data();

  imu_name = vio_dataset->get_imu_name();

  if (vio_dataset->get_accel_data().empty() ||
      vio_dataset->get_gyro_data().empty()) {
    std::cerr << "Error: dataset contains no IMU data. Does the dataset have "
                 "an IMU, and does a pattern match it?"
              << std::endl;
    std::abort();
  }

  // load allan deviations if they exist
  {
    std::string path = allanCachePath();

    std::ifstream is(path, std::ios::binary);

    // Reuse old caches without IMU name in the file name if the cache is unambiguous.
    if (!is.good() && !imu_name.empty() && sensor_filter.empty()) {
      const std::string legacy_path = allanCachePath(true);
      is.open(legacy_path, std::ios::binary);
      if (is.good()) {
        std::cout << "Reusing legacy allan-deviation cache: " << legacy_path << std::endl;
        path = legacy_path;
      }
    }

    if (is.good()) {
      cereal::BinaryInputArchive archive(is);

      archive(allan_deviations);

      std::cout << "Loaded allan deviations from: " << path << std::endl;
    } else {
      std::cout << "No pre-processed allan deviations found" << std::endl;
    }
  }

  // load calibration if exist
  {
    if (!calib) calib.reset(new Calibration<double>);

    std::ifstream is(cache_path + "calibration.json");
    if (is.good()) {
      cereal::JSONInputArchive archive(is);
      calib.reset(new Calibration<double>);
      archive(*calib);
      std::cout << "Loaded calibration from: " << cache_path
                << "calibration.json" << std::endl;
    }
  }

  // setup computor
  avar_computor = std::make_unique<AllanVarianceComputor>(
      vio_dataset, calib, allan_deviations, wn_min, wn_max, rr_min, rr_max,
      period_min, period_max);

  double gyro_rate = calculateRate(vio_dataset->get_gyro_data());
  double accel_rate = calculateRate(vio_dataset->get_accel_data());

  calib->imu_update_rate = gyro_rate;
  std::cout << "IMU rate: " << gyro_rate << std::endl;

  if (gyro_rate != accel_rate) {
    std::cerr << "Gyroscope and accelerometer run with different frequencies, "
                 "which is not supported."
              << std::endl;
  }

  double data_duration =
      calculateCaptureTimeNs(vio_dataset->get_gyro_data()) * 1e-9;
  if (data_duration < period_max) {
    std::cerr << "The dataset duration (" << data_duration
              << "s) must be at least the maximum period (" << period_max
              << "s) for computation." << std::endl;
  }

  if (show_gui) {
    recomputeDataLog();
    drawPlots();
  }
}

void ImuCalib::compute() {
  if (!calib) {
    std::cerr << "Initalize optimization first!" << std::endl;
    return;
  }
  std::cout << "Started computing variances" << std::endl;

  avar_computor->compute_deviations();
  AllanDeviations::Ptr new_allan_deviations = avar_computor->get_deviations();

  std::cout << "Done computing variances." << std::endl;

  if (!allan_deviations || new_allan_deviations->deviations.cols() >
                               allan_deviations->deviations.cols()) {
    allan_deviations = new_allan_deviations;

    std::string path = allanCachePath();
    std::ofstream os(path, std::ios::binary);
    cereal::BinaryOutputArchive archive(os);

    archive(allan_deviations);

    std::cout << "Saved them here: " << path << std::endl;
  }

  if (show_gui) {
    recomputeDataLog();
    drawPlots();
  }
}

void ImuCalib::fitLines() {
  if (!avar_computor || !avar_computor->is_computation_complete()) {
    std::cerr << "Compute allan deviations first!" << std::endl;
    return;
  }

  avar_computor->fit_lines(wn_min, wn_max, rr_min, rr_max);

  if (show_gui) {
    recomputeDataLog();
    drawPlots();
  }
}

void ImuCalib::saveCalib() {
  if (calib) {
    std::ofstream os(cache_path + "calibration.json");
    cereal::JSONOutputArchive archive(os);

    archive(*calib);

    std::cout << "Saved calibration in " << cache_path << "calibration.json"
              << std::endl;
  }
}

void ImuCalib::recomputeDataLog() {
  imu_raw_log.Clear();
  imu_calib_log.Clear();

  if (!vio_dataset) return;

  const auto &gyro_data = vio_dataset->get_gyro_data();
  const auto &accel_data = vio_dataset->get_accel_data();

  double min_time = gyro_data.front().timestamp_ns * 1e-9;
  imu_raw_time_max = gyro_data.back().timestamp_ns * 1e-9 - min_time;

  imu_raw_gyro_min = std::numeric_limits<double>::max();
  imu_raw_gyro_max = std::numeric_limits<double>::lowest();
  imu_raw_accel_min = std::numeric_limits<double>::max();
  imu_raw_accel_max = std::numeric_limits<double>::lowest();

  const size_t step = gyro_data.size() / LOG_DATAPOINTS;
  size_t next_step = step;
  Eigen::Vector3d gyro_sum, accel_sum;
  Eigen::Vector3d gyro_sum_step, accel_sum_step;
  gyro_sum.setZero();
  accel_sum.setZero();
  gyro_sum_step.setZero();
  accel_sum_step.setZero();

  std::vector<std::vector<float>> vals_uncentered;

  for (size_t i = 0; i < gyro_data.size(); ++i) {
    const basalt::GyroData &gd = gyro_data[i];
    const basalt::AccelData &ad = accel_data[i];

    if (i == next_step) {
      const auto gyro_avg = gyro_sum_step / step;
      const auto accel_avg = accel_sum_step / step;

      std::vector<float> vals;
      vals.reserve(7);
      double t = ad.timestamp_ns * 1e-9 - min_time;
      vals.push_back(t);

      for (int k = 0; k < 3; k++) vals.push_back(gyro_avg[k]);
      for (int k = 0; k < 3; k++) vals.push_back(accel_avg[k]);
      vals_uncentered.push_back(vals);

      imu_raw_gyro_min = std::min(imu_raw_gyro_min, gyro_avg.minCoeff());
      imu_raw_gyro_max = std::max(imu_raw_gyro_max, gyro_avg.maxCoeff());
      imu_raw_accel_min = std::min(imu_raw_accel_min, accel_avg.minCoeff());
      imu_raw_accel_max = std::max(imu_raw_accel_max, accel_avg.maxCoeff());

      next_step += step;
      gyro_sum += gyro_sum_step;
      accel_sum += accel_sum_step;
      gyro_sum_step.setZero();
      accel_sum_step.setZero();
    }

    gyro_sum_step += gd.data;
    accel_sum_step += ad.data;
  }

  imu_raw_gyro_centered_min = std::numeric_limits<float>::max();
  imu_raw_gyro_centered_max = std::numeric_limits<float>::lowest();
  imu_raw_accel_centered_min = std::numeric_limits<float>::max();
  imu_raw_accel_centered_max = std::numeric_limits<float>::lowest();

  const auto gyro_avg = gyro_sum / gyro_data.size();
  const auto accel_avg = accel_sum / accel_data.size();
  for (auto &vals : vals_uncentered) {
    vals.push_back(vals[1] - gyro_avg[0]);
    vals.push_back(vals[2] - gyro_avg[1]);
    vals.push_back(vals[3] - gyro_avg[2]);
    vals.push_back(vals[4] - accel_avg[0]);
    vals.push_back(vals[5] - accel_avg[1]);
    vals.push_back(vals[6] - accel_avg[2]);
    imu_raw_log.Log(vals);
    imu_raw_gyro_centered_min =
        std::min<double>(imu_raw_gyro_centered_min, vals[7]);
    imu_raw_gyro_centered_min =
        std::min<double>(imu_raw_gyro_centered_min, vals[8]);
    imu_raw_gyro_centered_min =
        std::min<double>(imu_raw_gyro_centered_min, vals[9]);
    imu_raw_gyro_centered_max =
        std::max<double>(imu_raw_gyro_centered_max, vals[7]);
    imu_raw_gyro_centered_max =
        std::max<double>(imu_raw_gyro_centered_max, vals[8]);
    imu_raw_gyro_centered_max =
        std::max<double>(imu_raw_gyro_centered_max, vals[9]);
    imu_raw_accel_centered_min =
        std::min<double>(imu_raw_accel_centered_min, vals[10]);
    imu_raw_accel_centered_min =
        std::min<double>(imu_raw_accel_centered_min, vals[11]);
    imu_raw_accel_centered_min =
        std::min<double>(imu_raw_accel_centered_min, vals[12]);
    imu_raw_accel_centered_max =
        std::max<double>(imu_raw_accel_centered_max, vals[10]);
    imu_raw_accel_centered_max =
        std::max<double>(imu_raw_accel_centered_max, vals[11]);
    imu_raw_accel_centered_max =
        std::max<double>(imu_raw_accel_centered_max, vals[12]);
  }

  imu_calib_gyro_min = std::numeric_limits<float>::max();
  imu_calib_gyro_max = std::numeric_limits<float>::lowest();
  imu_calib_accel_min = std::numeric_limits<float>::max();
  imu_calib_accel_max = std::numeric_limits<float>::lowest();

  if (avar_computor) {
    const auto calib_data = avar_computor->compute_data_log();

    const double step =
        (std::log10(period_max) - std::log10(period_min)) / LOG_DATAPOINTS;
    double next_step = std::log10(period_min);
    size_t sum_count = 0;
    std::vector<float> vals_sum(7, 0);
    for (const auto &vals : calib_data) {
      for (uint8_t i = 1; i < 7; i++) {
        vals_sum[i] += vals[i];
      }
      ++sum_count;

      if (vals[0] >= next_step) {
        vals_sum[0] = vals[0];
        for (uint8_t i = 1; i < 7; i++) {
          vals_sum[i] /= sum_count;
        }

        imu_calib_log.Log(vals);
        imu_calib_gyro_min = std::min(imu_calib_gyro_min, vals_sum[1]);
        imu_calib_gyro_min = std::min(imu_calib_gyro_min, vals_sum[2]);
        imu_calib_gyro_min = std::min(imu_calib_gyro_min, vals_sum[3]);
        imu_calib_gyro_max = std::max(imu_calib_gyro_max, vals_sum[1]);
        imu_calib_gyro_max = std::max(imu_calib_gyro_max, vals_sum[2]);
        imu_calib_gyro_max = std::max(imu_calib_gyro_max, vals_sum[3]);
        imu_calib_accel_min = std::min(imu_calib_accel_min, vals_sum[4]);
        imu_calib_accel_min = std::min(imu_calib_accel_min, vals_sum[5]);
        imu_calib_accel_min = std::min(imu_calib_accel_min, vals_sum[6]);
        imu_calib_accel_max = std::max(imu_calib_accel_max, vals_sum[4]);
        imu_calib_accel_max = std::max(imu_calib_accel_max, vals_sum[5]);
        imu_calib_accel_max = std::max(imu_calib_accel_max, vals_sum[6]);

        for (uint8_t i = 1; i < 7; i++) {
          vals_sum[i] = 0;
        }
        sum_count = 0;
        next_step += step;
      }
    }
  }
}

void ImuCalib::drawPlots() {
  plotter_raw->ClearSeries();
  plotter_raw->ClearMarkers();
  plotter_calib->ClearSeries();
  plotter_calib->ClearMarkers();

  double plotter_raw_min = std::numeric_limits<double>::max();
  double plotter_raw_max = std::numeric_limits<double>::lowest();
  float plotter_calib_min = std::numeric_limits<float>::max();
  float plotter_calib_max = std::numeric_limits<float>::lowest();

  if (show_gyro) {
    if (show_data) {
      if (center_data) {
        plotter_raw_min = std::min(plotter_raw_min, imu_raw_gyro_centered_min);
        plotter_raw_max = std::max(plotter_raw_max, imu_raw_gyro_centered_max);

        plotter_raw->AddSeries("$0", "$7", pangolin::DrawingModeLine,
                               pangolin::Colour::Red(), "g x");
        plotter_raw->AddSeries("$0", "$8", pangolin::DrawingModeLine,
                               pangolin::Colour::Green(), "g y");
        plotter_raw->AddSeries("$0", "$9", pangolin::DrawingModeLine,
                               pangolin::Colour::Blue(), "g z");
      } else {
        plotter_raw_min = std::min(plotter_raw_min, imu_raw_gyro_min);
        plotter_raw_max = std::max(plotter_raw_max, imu_raw_gyro_max);

        plotter_raw->AddSeries("$0", "$1", pangolin::DrawingModeLine,
                               pangolin::Colour::Red(), "g x");
        plotter_raw->AddSeries("$0", "$2", pangolin::DrawingModeLine,
                               pangolin::Colour::Green(), "g y");
        plotter_raw->AddSeries("$0", "$3", pangolin::DrawingModeLine,
                               pangolin::Colour::Blue(), "g z");
      }
    }

    plotter_calib_min = std::min(plotter_calib_min, imu_calib_gyro_min);
    plotter_calib_max = std::max(plotter_calib_max, imu_calib_gyro_max);

    plotter_calib->AddSeries("$0", "$1", pangolin::DrawingModeLine,
                             pangolin::Colour::Red(), "g x");
    plotter_calib->AddSeries("$0", "$2", pangolin::DrawingModeLine,
                             pangolin::Colour::Green(), "g y");
    plotter_calib->AddSeries("$0", "$3", pangolin::DrawingModeLine,
                             pangolin::Colour::Blue(), "g z");
    if (show_wn) {
      plotter_calib->AddSeries("$0", "$7", pangolin::DrawingModeLine,
                               pangolin::Colour(1.0f, 0.0f, 0.5f), "g x wn");
      plotter_calib->AddSeries("$0", "$8", pangolin::DrawingModeLine,
                               pangolin::Colour(0.5, 0.5, 0.5), "g y wn");
      plotter_calib->AddSeries("$0", "$9", pangolin::DrawingModeLine,
                               pangolin::Colour(0.5f, 0.0f, 1.0f), "g z wn");
    }
    if (show_rr) {
      plotter_calib->AddSeries("$0", "$10", pangolin::DrawingModeLine,
                               pangolin::Colour(1.0, 0.5, 0.0), "g x rr");
      plotter_calib->AddSeries("$0", "$11", pangolin::DrawingModeLine,
                               pangolin::Colour(0.5, 1.0, 0.0), "g y rr");
      plotter_calib->AddSeries("$0", "$12", pangolin::DrawingModeLine,
                               pangolin::Colour(0.5, 0.5, 0.5), "g z rr");
    }
  }

  if (show_accel) {
    if (show_data) {
      if (center_data) {
        plotter_raw_min = std::min(plotter_raw_min, imu_raw_accel_centered_min);
        plotter_raw_max = std::max(plotter_raw_max, imu_raw_accel_centered_max);

        plotter_raw->AddSeries("$0", "$10", pangolin::DrawingModeLine,
                               pangolin::Colour::Red(), "a x");
        plotter_raw->AddSeries("$0", "$11", pangolin::DrawingModeLine,
                               pangolin::Colour::Green(), "a y");
        plotter_raw->AddSeries("$0", "$12", pangolin::DrawingModeLine,
                               pangolin::Colour::Blue(), "a z");
      } else {
        plotter_raw_min = std::min(plotter_raw_min, imu_raw_accel_min);
        plotter_raw_max = std::max(plotter_raw_max, imu_raw_accel_max);

        plotter_raw->AddSeries("$0", "$4", pangolin::DrawingModeLine,
                               pangolin::Colour::Red(), "a x");
        plotter_raw->AddSeries("$0", "$5", pangolin::DrawingModeLine,
                               pangolin::Colour::Green(), "a y");
        plotter_raw->AddSeries("$0", "$6", pangolin::DrawingModeLine,
                               pangolin::Colour::Blue(), "a z");
      }
    }

    plotter_calib_min = std::min(plotter_calib_min, imu_calib_accel_min);
    plotter_calib_max = std::max(plotter_calib_max, imu_calib_accel_max);

    plotter_calib->AddSeries("$0", "$4", pangolin::DrawingModeLine,
                             pangolin::Colour::Red(), "a x");
    plotter_calib->AddSeries("$0", "$5", pangolin::DrawingModeLine,
                             pangolin::Colour::Green(), "a y");
    plotter_calib->AddSeries("$0", "$6", pangolin::DrawingModeLine,
                             pangolin::Colour::Blue(), "a z");
    if (show_wn) {
      plotter_calib->AddSeries("$0", "$13", pangolin::DrawingModeLine,
                               pangolin::Colour(1.0f, 0.0f, 0.5f), "a x wn");
      plotter_calib->AddSeries("$0", "$14", pangolin::DrawingModeLine,
                               pangolin::Colour(0.5, 0.5, 0.5), "a y wn");
      plotter_calib->AddSeries("$0", "$15", pangolin::DrawingModeLine,
                               pangolin::Colour(0.5f, 0.0f, 1.0f), "a z wn");
    }
    if (show_rr) {
      plotter_calib->AddSeries("$0", "$16", pangolin::DrawingModeLine,
                               pangolin::Colour(1.0, 0.5, 0.0), "a x rr");
      plotter_calib->AddSeries("$0", "$17", pangolin::DrawingModeLine,
                               pangolin::Colour(0.5, 1.0, 0.0), "a y rr");
      plotter_calib->AddSeries("$0", "$18", pangolin::DrawingModeLine,
                               pangolin::Colour(0.5, 0.5, 0.5), "a z rr");
    }
  }

  if (show_gyro || show_accel) {
    if (show_data) {
      plotter_raw->SetView(pangolin::XYRangef(
          0.0, imu_raw_time_max,
          plotter_raw_min - 0.05 * std::abs(plotter_raw_min),
          plotter_raw_max + 0.05 * std::abs(plotter_raw_max)));
    }

    if (plotter_calib_min < std::numeric_limits<float>::max() &&
        plotter_calib_max > std::numeric_limits<float>::lowest()) {
      plotter_calib->SetView(pangolin::XYRangef(
          std::log10(period_min) - 0.1, std::log10(period_max) + 0.1,
          plotter_calib_min - 0.1, plotter_calib_max + 0.1));

      if (show_wn) {
        plotter_calib->AddMarker(pangolin::Marker::Direction::Vertical,
                                 std::log10(1),
                                 pangolin::Marker::Equality::Equal,
                                 pangolin::Colour(1.0f, 0.0f, 1.0f));

        plotter_calib->AddMarker(pangolin::Marker::Direction::Vertical,
                                 std::log10(wn_min),
                                 pangolin::Marker::Equality::Equal,
                                 pangolin::Colour(0.5f, 0.0f, 0.5f));

        plotter_calib->AddMarker(pangolin::Marker::Direction::Vertical,
                                 std::log10(wn_max),
                                 pangolin::Marker::Equality::Equal,
                                 pangolin::Colour(0.5f, 0.0f, 0.5f));
      }
      if (show_rr) {
        plotter_calib->AddMarker(pangolin::Marker::Direction::Vertical,
                                 std::log10(3),
                                 pangolin::Marker::Equality::Equal,
                                 pangolin::Colour(1.0f, 1.0f, 0.0f));

        plotter_calib->AddMarker(pangolin::Marker::Direction::Vertical,
                                 std::log10(rr_min),
                                 pangolin::Marker::Equality::Equal,
                                 pangolin::Colour(0.5f, 0.5f, 0.0f));
        plotter_calib->AddMarker(pangolin::Marker::Direction::Vertical,
                                 std::log10(rr_max),
                                 pangolin::Marker::Equality::Equal,
                                 pangolin::Colour(0.5f, 0.5f, 0.0f));
      }
    }
  }
}

}  // namespace basalt
