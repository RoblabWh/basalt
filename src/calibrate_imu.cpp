#include <basalt/optimization/spline_optimize.h>

#include <basalt/calibration/imu_calib.h>

#include <CLI/CLI.hpp>

int main(int argc, char **argv) {
  std::string dataset_path;
  std::string dataset_type;
  std::string aprilgrid_path;
  std::string result_path;
  std::string cache_dataset_name;

  // Default values from https://github.com/ori-drs/allan_variance_ros
  double wn_min = 0.1, wn_max = 10.;
  double rr_min = 0.1, rr_max = 1000.;
  double period_min = 0.1, period_max = 1000.;

  CLI::App app{"Calibrate IMU"};

  app.add_option("--dataset-path", dataset_path, "Path to dataset")->required();
  app.add_option("--result-path", result_path, "Path to result folder")
      ->required();
  app.add_option("--dataset-type", dataset_type, "Dataset type (euroc, bag)")
      ->required();

  app.add_option("--cache-name", cache_dataset_name,
                 "Name to save cached files");

  app.add_option("--wn-min", wn_min, "Start of interval for white noise");
  app.add_option("--wn-max", wn_max, "End of interval for white noise");
  app.add_option("--rr-min", rr_min, "Start of interval for random walk");
  app.add_option("--rr-max", rr_max, "End of interval for random walk");
  app.add_option("--period-min", period_min, "Start of interval for allan plot");
  app.add_option("--period-max", period_max, "End of interval for allan plot");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError &e) {
    return app.exit(e);
  }

  if (cache_dataset_name.empty())
    cache_dataset_name = dataset_path.substr(dataset_path.rfind('/') + 1);

  basalt::ImuCalib cv(dataset_path, dataset_type, result_path,
                      cache_dataset_name, wn_min, wn_max, rr_min, rr_max, period_min, period_max);

  cv.renderingLoop();

  return 0;
}
