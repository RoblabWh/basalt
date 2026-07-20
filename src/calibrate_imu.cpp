#include <basalt/optimization/spline_optimize.h>

#include <basalt/calibration/imu_calib.h>

#include <CLI/CLI.hpp>

int main(int argc, char **argv) {
  basalt::ImuCalibOptions opts;
  bool headless = false;

  CLI::App app{"Calibrate IMU"};

  app.add_option("--dataset-path", opts.dataset_path, "Path to dataset")
      ->required();
  app.add_option("--result-path", opts.cache_path, "Path to result folder")
      ->required();
  app.add_option("--dataset-type", opts.dataset_type,
                 "Dataset type (euroc, bag)")
      ->required();

  app.add_option("--cache-name", opts.cache_dataset_name,
                 "Name to save cached files");

  app.add_option("--include", opts.sensor_include,
                 "Only use sensors matching one of these glob patterns")
      ->delimiter(',');
  app.add_option("--exclude", opts.sensor_exclude,
                 "Exclude sensors matching one of these glob patterns; "
                 "applied after --include")
      ->delimiter(',');

  app.add_option("--wn-min", opts.wn_min, "Start of interval for white noise");
  app.add_option("--wn-max", opts.wn_max, "End of interval for white noise");
  app.add_option("--rr-min", opts.rr_min, "Start of interval for random walk");
  app.add_option("--rr-max", opts.rr_max, "End of interval for random walk");
  app.add_option("--period-min", opts.period_min,
                 "Start of interval for allan plot");
  app.add_option("--period-max", opts.period_max,
                 "End of interval for allan plot");

  app.add_flag("--headless", headless, "Run calibration without GUI and exit");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError &e) {
    return app.exit(e);
  }

  opts.show_gui = !headless;

  if (opts.cache_dataset_name.empty())
    opts.cache_dataset_name =
        opts.dataset_path.substr(opts.dataset_path.rfind('/') + 1);

  basalt::ImuCalib cv(opts);

  if (headless) {
    return cv.runHeadless();
  }

  cv.renderingLoop();
  return 0;
}
