#include <basalt/calibration/imu_calib.h>

#include <basalt/utils/system_utils.h>

#include <basalt/serialization/headers_serialization.h>

#include <basalt/calibration/allan_variance.h>

namespace basalt {

ImuCalib::ImuCalib(const std::string &dataset_path,
                   const std::string &dataset_type,
                   const std::string &cache_path,
                   const std::string &cache_dataset_name, bool show_gui)
    : dataset_path(dataset_path),
      dataset_type(dataset_type),
      cache_path(ensure_trailing_slash(cache_path)),
      cache_dataset_name(cache_dataset_name),
      show_gui(show_gui),
      show_data("ui.show_data", false, false, true),
      show_accel("ui.show_accel", true, false, true),
      show_gyro("ui.show_gyro", true, false, true),
      load_dataset("ui.load_dataset", std::bind(&ImuCalib::loadDataset, this)),
      show_wn("ui.show_wn", true, false, true),
      show_rr("ui.show_rr", true, false, true),
      init_opt("ui.init_opt", std::bind(&ImuCalib::initOptimization, this)),
      comp("ui.compute", std::bind(&ImuCalib::compute, this)),
      save_calib("ui.save_calib", std::bind(&ImuCalib::saveCalib, this)) {
  if (show_gui) initGui();
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
                                      0.01f, 0.01f);
  plot_raw_display.AddDisplay(*plotter_raw);
  plotter_calib =
      new pangolin::Plotter(&imu_calib_log, -1.0, 3.0, -8.0, 0.0, 0.1, 0.1);
  plot_calib_display.AddDisplay(*plotter_calib);
}

void ImuCalib::renderingLoop() {
  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (vio_dataset) {
      if (show_data.GuiChanged() || show_accel.GuiChanged() ||
          show_gyro.GuiChanged() || show_wn.GuiChanged() ||
          show_rr.GuiChanged()) {
        drawPlots();
      }
    }

    pangolin::FinishFrame();
  }
}

void ImuCalib::loadDataset() {
  basalt::DatasetIoInterfacePtr dataset_io =
      basalt::DatasetIoFactory::getDatasetIo(dataset_type);

  dataset_io->read(dataset_path);

  vio_dataset = dataset_io->get_data();

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

  if (show_gui) {
    recomputeDataLog();
    drawPlots();
  }
}

void ImuCalib::initOptimization() {
  if (!vio_dataset) {
    std::cerr << "Load dataset first!" << std::endl;
    return;
  }

  avar_computor = std::make_unique<AllanVarianceComputor>(vio_dataset, calib);

  double gyro_rate = calculateRate(vio_dataset->get_gyro_data());
  double accel_rate = calculateRate(vio_dataset->get_accel_data());

  if (gyro_rate != accel_rate) {
    std::cerr << "Gyroscope and accelerometer run with different frequencies, "
                 "which is not supported."
              << std::endl;
  }

  if (calculateCaptureTimeNs(vio_dataset->get_gyro_data()) < 1e12) {
    std::cerr << "Minimum duration of 1000s (~3h) required for computation."
              << std::endl;
  }

  std::cout << "IMU rate: " << gyro_rate << std::endl;

  calib->imu_update_rate = gyro_rate;

  if (show_gui) recomputeDataLog();

  std::cout << "Initialized optimization." << std::endl;
}

void ImuCalib::compute() {
  if (!calib) {
    std::cerr << "Initalize optimization first!" << std::endl;
    return;
  }
  std::cout << "Started computing variances" << std::endl;

  avar_computor->compute();

  std::cout << "Done computing variances" << std::endl;
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

  for (size_t i = 0; i < gyro_data.size(); ++i) {
    const basalt::GyroData &gd = gyro_data[i];
    const basalt::AccelData &ad = accel_data[i];

    std::vector<float> vals;
    vals.reserve(7);
    double t = ad.timestamp_ns * 1e-9 - min_time;
    vals.push_back(t);

    for (int k = 0; k < 3; k++) vals.push_back(gd.data[k]);
    for (int k = 0; k < 3; k++) vals.push_back(ad.data[k]);

    imu_raw_log.Log(vals);
  }

  if (avar_computor) {
    const auto calib_data = avar_computor->compute_data_log();
    for (const auto &vals : calib_data) imu_calib_log.Log(vals);
  }
}

void ImuCalib::drawPlots() {
  plotter_raw->ClearSeries();
  plotter_raw->ClearMarkers();
  plotter_calib->ClearSeries();
  plotter_calib->ClearMarkers();

  if (show_gyro) {
    if (show_data) {
      plotter_raw->AddSeries("$0", "$1", pangolin::DrawingModeLine,
                             pangolin::Colour::Red(), "g x");
      plotter_raw->AddSeries("$0", "$2", pangolin::DrawingModeLine,
                             pangolin::Colour::Green(), "g y");
      plotter_raw->AddSeries("$0", "$3", pangolin::DrawingModeLine,
                             pangolin::Colour::Blue(), "g z");
    }

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
      plotter_calib->AddMarker(pangolin::Marker::Direction::Vertical,
                               std::log10(1), pangolin::Marker::Equality::Equal,
                               pangolin::Colour(1.0f, 0.0f, 1.0f));
    }
    if (show_rr) {
      plotter_calib->AddSeries("$0", "$10", pangolin::DrawingModeLine,
                               pangolin::Colour(1.0, 0.5, 0.0), "g x rr");
      plotter_calib->AddSeries("$0", "$11", pangolin::DrawingModeLine,
                               pangolin::Colour(0.5, 1.0, 0.0), "g y rr");
      plotter_calib->AddSeries("$0", "$12", pangolin::DrawingModeLine,
                               pangolin::Colour(0.5, 0.5, 0.5), "g z rr");
      plotter_calib->AddMarker(pangolin::Marker::Direction::Vertical,
                               std::log10(3), pangolin::Marker::Equality::Equal,
                               pangolin::Colour(1.0f, 1.0f, 0.0f));
    }
  }

  if (show_accel) {
    if (show_data) {
      plotter_raw->AddSeries("$0", "$4", pangolin::DrawingModeLine,
                             pangolin::Colour::Red(), "a x");
      plotter_raw->AddSeries("$0", "$5", pangolin::DrawingModeLine,
                             pangolin::Colour::Green(), "a y");
      plotter_raw->AddSeries("$0", "$6", pangolin::DrawingModeLine,
                             pangolin::Colour::Blue(), "a z");
    }

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
      plotter_calib->AddMarker(pangolin::Marker::Direction::Vertical,
                               std::log10(1), pangolin::Marker::Equality::Equal,
                               pangolin::Colour(1.0f, 0.0f, 1.0f));
    }
    if (show_rr) {
      plotter_calib->AddSeries("$0", "$16", pangolin::DrawingModeLine,
                               pangolin::Colour(1.0, 0.5, 0.0), "a x rr");
      plotter_calib->AddSeries("$0", "$17", pangolin::DrawingModeLine,
                               pangolin::Colour(0.5, 1.0, 0.0), "a y rr");
      plotter_calib->AddSeries("$0", "$18", pangolin::DrawingModeLine,
                               pangolin::Colour(0.5, 0.5, 0.5), "a z rr");
      plotter_calib->AddMarker(pangolin::Marker::Direction::Vertical,
                               std::log10(3), pangolin::Marker::Equality::Equal,
                               pangolin::Colour(1.0f, 1.0f, 0.0f));
    }
  }
}

}  // namespace basalt
