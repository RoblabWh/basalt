#include <basalt/optimization/spline_optimize.h>

#include <basalt/calibration/imu_calib.h>

#include <CLI/CLI.hpp>

int main(int argc, char **argv) {
  std::string dataset_path;
  std::string dataset_type;
  std::string aprilgrid_path;
  std::string result_path;
  std::string cache_dataset_name;

  CLI::App app{"Calibrate IMU"};

  app.add_option("--dataset-path", dataset_path, "Path to dataset")->required();
  app.add_option("--result-path", result_path, "Path to result folder")
      ->required();
  app.add_option("--dataset-type", dataset_type, "Dataset type (euroc, bag)")
      ->required();

  app.add_option("--cache-name", cache_dataset_name,
                 "Name to save cached files");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError &e) {
    return app.exit(e);
  }

  if (cache_dataset_name.empty())
    cache_dataset_name = dataset_path.substr(dataset_path.rfind('/') + 1);

  basalt::ImuCalib cv(dataset_path, dataset_type, result_path,
                      cache_dataset_name);

  cv.renderingLoop();

  return 0;
}
