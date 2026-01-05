## Basalt
Fork of https://gitlab.com/VladyslavUsenko/basalt.git

For more information see https://vision.in.tum.de/research/vslam/basalt

![teaser](doc/img/teaser.png)

This project contains tools for:
* Camera, IMU and motion capture calibration.
* Visual-inertial odometry and mapping.
* Simulated environment to test different components of the system.

Some reusable components of the system are available as a separate [header-only library](https://github.com/RoblabWh/basalt-headers) (Fork of https://gitlab.com/VladyslavUsenko/basalt-headers).

## Related Publications
Visual-Inertial Odometry and Mapping:
* **Visual-Inertial Mapping with Non-Linear Factor Recovery**, V. Usenko, N. Demmel, D. Schubert, J. Stückler, D. Cremers, In IEEE Robotics and Automation Letters (RA-L) [[DOI:10.1109/LRA.2019.2961227]](https://doi.org/10.1109/LRA.2019.2961227) [[arXiv:1904.06504]](https://arxiv.org/abs/1904.06504).

Calibration (explains implemented camera models):
* **The Double Sphere Camera Model**, V. Usenko and N. Demmel and D. Cremers, In 2018 International Conference on 3D Vision (3DV), [[DOI:10.1109/3DV.2018.00069]](https://doi.org/10.1109/3DV.2018.00069), [[arXiv:1807.08957]](https://arxiv.org/abs/1807.08957).

Calibration (demonstrates how these tools can be used for dataset calibration):
* **The TUM VI Benchmark for Evaluating Visual-Inertial Odometry**, D. Schubert, T. Goll,  N. Demmel, V. Usenko, J. Stückler, D. Cremers, In 2018 International Conference on Intelligent Robots and Systems (IROS), [[DOI:10.1109/IROS.2018.8593419]](https://doi.org/10.1109/IROS.2018.8593419), [[arXiv:1804.06120]](https://arxiv.org/abs/1804.06120).

Calibration (describes B-spline trajectory representation used in camera-IMU calibration):
* **Efficient Derivative Computation for Cumulative B-Splines on Lie Groups**, C. Sommer, V. Usenko, D. Schubert, N. Demmel, D. Cremers, In 2020 Conference on Computer Vision and Pattern Recognition (CVPR), [[DOI:10.1109/CVPR42600.2020.01116]](https://doi.org/10.1109/CVPR42600.2020.01116), [[arXiv:1911.08860]](https://arxiv.org/abs/1911.08860).

Optimization (describes square-root optimization and marginalization used in VIO/VO):
* **Square Root Marginalization for Sliding-Window Bundle Adjustment**, N. Demmel, D. Schubert, C. Sommer, D. Cremers, V. Usenko, In 2021 International Conference on Computer Vision (ICCV), [[arXiv:2109.02182]](https://arxiv.org/abs/2109.02182)


## Installation
Clone the project's source code:
```
git clone --recursive https://github.com/RoblabWh/basalt
cd basalt
```

### Docker Container
Build the Docker image and run a container:
```
docker build -t basalt .
xhost +local:
docker run -it --rm --env=DISPLAY --env=QT_X11_NO_MITSHM=1 --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw --volume="$HOME":/home/basalt basalt
```

### Source installation for Ubuntu 18.04, 20.04, 22.04 and macOS >= 10.14 (Mojave)
Build and install the project. For macOS, you should have [Homebrew](https://brew.sh/) installed.
```
./scripts/install_deps.sh
cmake -B build -G Ninja -D CMAKE_BUILD_TYPE=Release
cmake --build build --parallel $(nproc)
sudo cmake --install build
```

## Usage
* [Camera, IMU and Mocap calibration. (TUM-VI, Euroc, UZH-FPV and Kalibr datasets)](doc/Calibration.md)
* [Visual-inertial odometry and mapping. (TUM-VI and Euroc datasets)](doc/VioMapping.md)
* [Visual odometry (no IMU). (KITTI dataset)](doc/Vo.md)
* [Simulation tools to test different components of the system.](doc/Simulation.md)
* [Batch evaluation tutorial (ICCV'21 experiments)](doc/BatchEvaluation.md)

## Device support
* [Tutorial on Camera-IMU and Motion capture calibration with Realsense T265.](doc/Realsense.md)

## Development
* [Development environment setup.](doc/DevSetup.md)

## Licence
The code is provided under a BSD 3-clause license. See the LICENSE file for details.
Note also the different licenses of thirdparty submodules.

Some improvements are ported back from the fork
[granite](https://github.com/DLR-RM/granite) (MIT license).
