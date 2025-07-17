# SLAM Lidar & Navigation

## Important Note
This repository is a copy of my original repo. For full commit history, context, and updates, please visit my source repository:

**[Original Repo Link: https://github.com/KaveeshwaraBandara/AMR_controller_SLAM.git ]**

I recommend users review the initial commit history and documentation in the original repository for deeper understanding and context.
This project implements Simultaneous Localization and Mapping (SLAM) and autonomous navigation using Lidar, IMU, and other sensors. It features modules for mapping, path planning (A*), sensor fusion (EKF), ICP-based scan matching, and trajectory visualization.

## Highlights
- **No frameworks used**: This project is built entirely with C++ standard libraries and does not use any frameworks.
- **External dependencies**: Only external libraries used are Eigen (for linear algebra) and OpenCV (for visualization). No other third-party frameworks are required.

## Features
- Real-time SLAM with Lidar and IMU
- Path planning (A* algorithm)
- Sensor fusion (EKF)
- ICP scan matching
- Trajectory and occupancy grid visualization

## Project Structure
- `main.cpp`: Entry point
- Source modules: `lidar/`, `imu/`, `mapping/`, `icp/`, `EKF/`, `Astar/`, `navigation/`, `SLAMUtils/`, `trajectoryVizualizer/`, `Communication/`
- External libraries: `External/eigen/` (Eigen, not included), `rplidar_sdk/` (S2L Lidar SDK, not included)

## Dependencies
### External Libraries
- **Eigen**: Linear algebra library. Not included in this repo. Download from [Eigen GitLab](https://gitlab.com/libeigen/eigen) and place in `External/eigen/`.
- **OpenCV**: For visualization. You must install OpenCV before building and running the project:
  ```bash
  sudo apt install libopencv-dev
  ```

### Lidar SDK
- **S2L Lidar SDK**: Required for Lidar communication. Not included in this repo. Download from the official S2L Lidar website and place in `rplidar_sdk/`.

## Build Instructions
1. Install dependencies (see above)
2. Build:
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```
3. Run:
   ```bash
   ./main
   ```

## License
See [LICENSE](LICENSE).
