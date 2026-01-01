# Autonomous-Mobile-Robot-SLAM-Navigation-on-2D-Environment

A Python-based robotics simulation environment demonstrating occupancy-grid mapping, LIDAR raycasting, A* path planning, and real-time telemetry/plotting. The project provides:

- A simple differential-drive robot simulator
- LIDAR raycasting against a truth map image
- An occupancy grid mapper using log-odds updates
- A* path planner with path smoothing and collision-inflation checks
- Real-time telemetry plots (velocity, angular rate, heading error, planning status)

See the code and examples in the repository for usage. Suggested next steps: add unit tests for the mapper and planner, and set up CI to run flake8/ruff and the test-suite.
