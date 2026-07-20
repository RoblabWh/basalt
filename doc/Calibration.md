# Calibration

Here, we explain how to use the calibration tools with the [TUM-VI](https://vision.in.tum.de/data/datasets/visual-inertial-dataset) dataset as an example.

Supported dataset formats include:
- euroc: [EuRoC MAV](https://projects.asl.ethz.ch/datasets/euroc-mav/) and [TUM VI](https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset)
- uzh: [UZH-FPV](https://fpv.ifi.uzh.ch/)
- kitti: [KITTI](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)
- bag: ROS1 and ROS2 bags with exposure times in nanoseconds encoded in the `frame_id` field of the header, as done in [TUM VI](https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset)
- cv: OpenCV VideoCapture for videos and image sequences - **ONLY** for geometric camera calibration (no exposure times or IMU data)

## TUM-VI dataset
Download the datasets for camera and camera-IMU calibration:
```
mkdir ~/tumvi_calib_data
cd ~/tumvi_calib_data
curl -LOk http://vision.in.tum.de/tumvi/raw/dataset-calib-cam3.bag
curl -LOk http://vision.in.tum.de/tumvi/raw/dataset-calib-imu1.bag
curl -LOk http://vision.in.tum.de/tumvi/imu_static/dataset-calib-imu-static2.bag
```

### Camera calibration
Run the camera calibration:
```
basalt_calibrate_cam --dataset-path ~/tumvi_calib_data/dataset-calib-cam3.bag --dataset-type bag --aprilgrid /usr/local/etc/basalt/aprilgrid_6x6.json --result-path ~/tumvi_calib_result/ --cam-types ds ds
```
The command line options have the following meaning:
* `--dataset-path` path to the dataset.
* `--dataset-type` type of the dataset.
* `--result-path` path to the folder where the resulting calibration and intermediate results will be stored.
* `--aprilgrid` path to the configuration file for the aprilgrid.
* `--cam-types` camera models for the image streams in the dataset. For more details see [arXiv:1807.08957](https://arxiv.org/abs/1807.08957).

### Selecting sensors

Each tool loads only the sensor *kinds* it needs: `basalt_calibrate_cam` reads cameras only, `basalt_calibrate_imu` reads the IMU only, and `basalt_calibrate_vi` reads cameras, IMU and ground truth. Sensors of other kinds are ignored entirely, so for example camera calibration runs fine on a dataset that also happens to contain several IMUs.

Within the kinds it loads, a tool uses every sensor found by default. `--include` and `--exclude` narrow this selection by name. Both take a list of glob patterns (`*`, `?`, `[...]`; repeatable or comma-separated). If `--include` is given, only sensors matching one of its patterns are used; `--exclude` is applied afterwards and removes sensors again. These options are available on all tools loading datasets.

`basalt_calibrate_imu` and `basalt_calibrate_vi` each need exactly one IMU. If the dataset contains more than one, the tool aborts and lists them; use `--include`/`--exclude` to narrow the selection down to a single IMU. `basalt_calibrate_vi` applies the same single-selection rule to ground-truth (mocap pose / position) streams.

The sensor names depend on the dataset type:
* `bag` (ROS1/ROS2): the full topic name, e.g. `/cam0/image_raw`, `/imu0`. Applies to image, IMU, mocap and position topics, so `--include` can also pin which IMU topic is used in a multi-IMU bag.
* `euroc`: `cam0`, `cam1`, `imu0`.
* `uzh`: `left`, `right`, `imu`.
* `kitti`: `image_0`, `image_1`.
* `dai`: the camera csv/folder name under `cams/`, plus `imu`.
* `cv`: the zero-based position in the video list, i.e. `0`, `1`, ...

After that, you should see the calibration GUI:
![calibration_cam](/doc/img/calibration_cam.png)

The buttons in the GUI are located in the order you should follow to calibrate the camera. After pressing a button the system will print the output to the command line:
* `load_dataset` loads the dataset.
* `detect_corners` starts corner detection in the background thread. Since it is the most time-consuming part of the calibration process, the detected corners are cached and loaded if you run the executable again pointing to the same result folder path.
* `init_cam_intr` computes an initial guess for camera intrinsics.
* `init_cam_poses` computes an initial guess for camera poses given the current intrinsics.
* `init_cam_extr` computes an initial transformation between the cameras.
* `init_opt` initializes optimization and shows the projected points given the current calibration and camera poses.
* `optimize` runs an iteration of the optimization and visualizes the result. You should press this button until the error printed in the console output stops decreasing and the optimization converges. Alternatively, you can use the `opt_until_converge` checkbox that will run the optimization until it converges automatically.
* `save_calib` saves the current calibration as `calibration.json` in the result folder.
* `compute_vign` **(Experimental)** computes a radially symmetric vignetting for the cameras. For the algorithm to work, **the calibration pattern should be static (camera moving around it) and have constant lighting throughout the calibration sequence**. If you run `compute_vign` you should press `save_calib` afterwards. The png images with vignetting will also be stored in the result folder.
* `set_lin_resp` sets the response function of the cameras to be perfectly linear. If you run `set_lin_resp` you should press `save_calib` afterwards.
* `compute_resp` computes the response function of the cameras. For the algorithm to work, **the camera has to be static and capture images in the full range of exposure, evenly distributed, while having constant lighting throughout the calibration sequence**. If you run `compute_resp` you should press `save_calib` afterwards. The txt files with the non-linear response function will also be stored in the result folder.

You can also control the process using the following buttons:
* `show_frame` slider to switch between the frames in the sequence.
* `show_corners` toggles the visibility of the detected corners shown in red.
* `show_corners_rejected` toggles the visibility of rejected corners. Works only when `show_corners` is enabled.
* `show_init_reproj` shows the initial reprojections computed by the `init_cam_poses` step.
* `show_opt` shows reprojected corners with the current estimate of the intrinsics and poses.
* `show_vign` toggles the visibility of the points used for vignetting estimation. The points are distributed across white areas of the pattern.
* `show_ids` toggles the ID visualization for every point.
* `huber_thresh` controls the threshold for the huber norm in pixels for the optimization.
* `opt_intr` controls if the optimization can change the intrinsics. For some datasets it might be helpful to disable this option for several first iterations of the optimization.
* `opt_until_converge` runs the optimization until convergence.
* `stop_thresh` defines the stopping criterion. Optimization will stop when the maximum increment is smaller than this value.

### IMU calibration
Run the IMU calibration:
```
basalt_calibrate_imu --dataset-path ~/tumvi_calib_data/dataset-calib-imu-static2.bag --dataset-type bag --result-path ~/tumvi_calib_result/ --period-min 0.005 --period-max 50000 --wn-min 0.02 --wn-max 1.0 --rr-min 1000.0 --rr-max 6000.0
```
The command line options have the following meaning:
* `--dataset-path` path to the dataset.
* `--dataset-type` type of the dataset.
* `--result-path` path to the folder where the resulting calibration and intermediate results will be stored.
* `--period-min` start period for allan plot. Result caching is dependent on this value and on the selected IMU (see `compute` below).
* `--period-max` end period for allan plot.
* `--wn-min` start period for white noise.
* `--wn-max` end period for white noise.
* `--rr-min` start period for random walk.
* `--rr-max` end period for random walk.

After that, you should see the calibration GUI:
![calibration_imu](/doc/img/calibration_imu.png)

The buttons in the GUI are located in the order you should follow to calibrate the IMU. After pressing a button the system will print the output to the command line:
* `load_dataset` loads the dataset.
* `compute` starts allan plot computation. It is the most time-consuming part of the calibration process. Results are cached in the result folder and depend on `--period-min`.
* `fit_lines` fits lines with slopes -1/2 (white noise) and 1/2 (random walk) into the allan plot for each sensor axis to estimate noise parameters.
* `save_calib` saves the current calibration as `calibration.json` in the result folder.

You can also control the process using the following buttons:
* `show_data` toggles the visibility of the input data.
* `center_data` toggles centering the input data to zero for better visibility.
* `show_accel` toggles the visibility of the accelerometer sensor in input and output plots.
* `show_gyro` toggles the visibility of the gyroscope sensor in input and output plots.
* `show_wn` toggles the visibility of the white noise data and lines for both sensors.
* `show_rr` toggles the visibility of the random walk data and lines for both sensors.
* `wn_min` slider to modify the `--wn-min` option.
* `wn_max` slider to modify the `--wn-max` option.
* `rr_min` slider to modify the `--rr-min` option.
* `rr_max` slider to modify the `--rr-max` option.

### Camera + IMU + Mocap calibration
After calibrating the camera and IMU you can run the camera + IMU + Mocap calibration. The result path should point to the **same folder as before**:
```
basalt_calibrate_vi --dataset-path ~/tumvi_calib_data/dataset-calib-imu1.bag --dataset-type bag --aprilgrid /usr/local/etc/basalt/aprilgrid_6x6.json --result-path ~/tumvi_calib_result/ --gyro-noise-std 0.000282 --accel-noise-std 0.016 --gyro-bias-std 0.0001 --accel-bias-std 0.001
```
The command line options for the IMU noise are continuous-time and defined as in [Kalibr](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model):
* `--gyro-noise-std` gyroscope white noise.
* `--accel-noise-std` accelerometer white noise.
* `--gyro-bias-std` gyroscope random walk.
* `--accel-bias-std` accelerometer random walk.

**NOTE:** These options provide default values and will be overridden by any noise parameters present in the `calibration.json` from previous IMU or VI calibrations.

`--cam-types` is accepted as an optional argument. The camera models themselves are taken from the `calibration.json` in the result folder, if `--cam-types` is set, it is checked against those models and the calibration aborts on a mismatch.

![calibration_vi](/doc/img/calibration_vi.png)

The buttons in the GUI are located in the order you need to follow to calibrate the camera-IMU setup:
* `load_dataset`, `detect_corners`, `init_cam_poses` same as above.
* `init_cam_imu` initializes the rotation between camera and IMU by aligning rotation velocities of the camera to the gyro data.
* `init_opt` initializes the optimization. Shows reprojected corners in magenta and the estimated values from the spline as solid lines below.
* `optimize` runs an iteration of the optimization. You should press it several times until convergence before proceeding to next steps. Alternatively, you can use the `opt_until_converge` checkbox that will run the optimization until it converges automatically.
* `init_mocap` initializes the transformation from the Aprilgrid calibration pattern to the Mocap coordinate system.
* `save_calib` saves the current calibration as `calibration.json` in the result folder.
* `save_mocap_calib` saves the current Mocap to IMU calibration as `mocap_calibration.json` in the result folder.

You can also control the visualization using the following buttons:
* `show_frame` - `show_ids` the same as above.
* `show_spline` toggles the visibility of enabled measurements (accel, gyro, position, velocity) generated from the spline that we optimize.
* `show_data` toggles the visibility of raw data contained in the dataset.
* `show_accel` shows accelerometer data.
* `show_gyro` shows gyroscope data.
* `show_pos` shows spline position for `show_spline` and positions generated from camera pose initialization transformed into the IMU coordinate frame for `show_data`.
* `show_rot_error` shows the rotation error between spline and camera pose initializations transformed into the IMU coordinate frame.
* `show_mocap` shows the mocap marker position transformed to the IMU frame.
* `show_mocap_rot_error` shows rotation between the spline and Mocap measurements.
* `show_mocap_rot_vel` shows the rotation velocity computed from the Mocap.

The following options control the optimization process:
* `opt_intr` enables optimization of intrinsics. Usually should be disabled for the camera-IMU calibration.
* `opt_poses` enables optimization based on camera pose initialization. Sometimes helps to better initialize the spline before running optimization with `opt_corners`.
* `opt_corners` enables optimization based on reprojection corner positions **(should be used by default)**.
* `opt_cam_time_offset` computes the time offset between camera and the IMU. This option should be used only for refinement when the optimization already converged.
* `opt_imu_scale` enables IMU axis scaling, rotation and misalignment calibration. This option should be used only for refinement when the optimization already converged.
* `opt_mocap` enables Mocap optimization. You should run it only after pressing `init_mocap`.
* `huber_thresh` controls the threshold for the huber norm in pixels for the optimization.
* `opt_until_convg` runs the optimization until convergence.
* `stop_thresh` defines the stopping criterion. Optimization will stop when the maximum increment is smaller than this value.

## Headless mode

All three calibration tools accept a `--headless` flag that runs the full calibration pipeline without the GUI and exits. The steps are executed in the same order as the GUI buttons described above, the optimization runs until convergence, and the result is saved to the result folder automatically:
```
basalt_calibrate_cam --dataset-path ~/tumvi_calib_data/dataset-calib-cam3.bag --dataset-type bag --aprilgrid /usr/local/etc/basalt/aprilgrid_6x6.json --result-path ~/tumvi_calib_result/ --cam-types ds ds --headless
basalt_calibrate_imu --dataset-path ~/tumvi_calib_data/dataset-calib-imu-static2.bag --dataset-type bag --result-path ~/tumvi_calib_result/ --period-min 0.005 --period-max 50000 --wn-min 0.02 --wn-max 1.0 --rr-min 1000.0 --rr-max 6000.0 --headless
basalt_calibrate_vi --dataset-path ~/tumvi_calib_data/dataset-calib-imu1.bag --dataset-type bag --aprilgrid /usr/local/etc/basalt/aprilgrid_6x6.json --result-path ~/tumvi_calib_result/ --headless
```

The GUI parameters described above are available as command line options (defaults match the GUI): `--opt-intr`, `--huber-thresh` and `--stop-thresh` for both `basalt_calibrate_cam` and `basalt_calibrate_vi`; `--opt-poses`, `--opt-cam-time-offset` and `--opt-imu-scale` for `basalt_calibrate_vi` only. In addition:
* `--max-iterations` limits the number of optimization iterations (default 100).
* `--compute-vignette` / `--compute-response` (`basalt_calibrate_cam`) run the experimental vignette / response estimation after the optimization, like the `compute_vign` / `compute_resp` buttons.
* `--save-mocap` (`basalt_calibrate_vi`) enables Mocap optimization and additionally saves `mocap_calibration.json`, like the `init_mocap` and `save_mocap_calib` buttons.

Exit codes: `0` on success, `1` if a pipeline step fails (e.g. no corners detected, or no previous camera calibration found for `basalt_calibrate_vi`), `2` if the optimization did not converge within `--max-iterations` (the current estimate is still saved).
