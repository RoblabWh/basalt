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

#include <basalt/calibration/cam_calib.h>

#include <CLI/CLI.hpp>

int main(int argc, char **argv) {
  basalt::CamCalibOptions opts;
  bool headless = false;
  int max_iterations = 100;
  bool compute_vignette = false;
  bool compute_response = false;

  CLI::App app{"Calibrate IMU"};

  app.add_option("--dataset-path", opts.dataset_path, "Path to dataset")
      ->required();
  app.add_option("--result-path", opts.cache_path, "Path to result folder")
      ->required();
  app.add_option("--dataset-type", opts.dataset_type,
                 "Dataset type (euroc, bag)")
      ->required();

  app.add_option("--aprilgrid", opts.aprilgrid_path,
                 "Path to Aprilgrid config file)")
      ->required();

  app.add_option("--cache-name", opts.cache_dataset_name,
                 "Name to save cached files");

  app.add_option("--cam-types", opts.cam_types,
                 "Type of cameras (eucm, ds, kb4, pinhole)")
      ->required();

  app.add_option("--include", opts.sensor_include,
                 "Only use sensors matching one of these glob patterns")
      ->delimiter(',');
  app.add_option("--exclude", opts.sensor_exclude,
                 "Exclude sensors matching one of these glob patterns; "
                 "applied after --include")
      ->delimiter(',');

  app.add_option("--skip-images", opts.skip_images, "Number of images to skip");
  app.add_option("--start-image", opts.start_image,
                 "Index of the first image to use");
  app.add_option(
      "--end-image", opts.end_image,
      "Index of the last image to use, negative means counting from the end");

  app.add_flag("--headless", headless, "Run calibration without GUI and exit");
  app.add_option("--opt-intr", opts.opt_intr, "Optimize camera intrinsics");
  app.add_option("--huber-thresh", opts.huber_thresh,
                 "Huber threshold for optimization (pixels)");
  app.add_option("--stop-thresh", opts.stop_thresh,
                 "Optimization convergence threshold");
  app.add_option("--max-iterations", max_iterations,
                 "Maximum number of optimization iterations (headless)");
  app.add_flag("--compute-vignette", compute_vignette,
               "Compute vignette after optimization (headless)");
  app.add_flag("--compute-response", compute_response,
               "Compute inverse response function after optimization "
               "(headless)");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError &e) {
    return app.exit(e);
  }

  opts.show_gui = !headless;

  if (opts.cache_dataset_name.empty())
    opts.cache_dataset_name =
        opts.dataset_path.substr(opts.dataset_path.rfind('/') + 1);

  basalt::CamCalib cv(opts);

  if (headless) {
    return cv.runHeadless(max_iterations, compute_vignette, compute_response);
  }

  cv.renderingLoop();
  return 0;
}
