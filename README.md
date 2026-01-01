## 2D SLAM and Path Planning Simulation

This repository implements a compact **2D robotic navigation simulation** that combines:
              - Simulated LIDAR sensing
              - Log-odds occupancy grid mapping
              - A* path planning with obstacle inflation
              - Path smoothing using line-of-sight checks
              - Unicycle kinematic robot model

The project is intended for **educational use, research simulation, and robotics education**.

---

## System Pipeline
1. Robot motion update using kinematic equations  
2. LIDAR raycasting against a ground-truth map  
3. Occupancy grid update using log-odds  
4. A* path planning on the grid  
5. Path smoothing  
6. Visualization and telemetry  

---

## Mathematical Models (Plain-Text Form)

### Robot Motion Model (Unicycle)
State: (x, y, theta)

Motion equations:
- x_dot = v * cos(theta)
- y_dot = v * sin(theta)
- theta_dot = omega

Discrete-time update:
- x_next = x + v * cos(theta) * dt
- y_next = y + v * sin(theta) * dt
- theta_next = theta + omega * dt

Optional Gaussian noise may be added to simulate odometry uncertainty.

---

### LIDAR Measurement Model
Each LIDAR beam measures a range value:

- z = min(z_true + noise, z_max)
- noise ~ Normal(0, sigma^2)

Raycasting is used to detect the first obstacle along each beam.

---

### Occupancy Grid Mapping (Log-Odds)
Each grid cell stores a log-odds value:

- L = log( P(occupied) / (1 - P(occupied)) )

Recursive update:
- L_new = L_old + inverse_sensor_model

Inverse sensor model:
- Cells along a beam before obstacle:
  - L += L_free
- Cell where obstacle is detected:
  - L += L_occ

Log-odds values are clamped:
- L_min <= L <= L_max

Occupancy probability (for visualization):
- P = 1 / (1 + exp(-L))

---

## Core Algorithms

### Occupancy Grid Update (Pseudocode)
```text
for each lidar beam:
    cells = bresenham(ray)
    for each cell before hit:
        L[cell] += L_free
    if obstacle detected:
        L[hit_cell] += L_occ
    clamp L[cell] to [L_min, L_max]
```

---

### A* Path Planning (Pseudocode)
```text
open_set = {start}
while open_set not empty:
    n = node with minimum f = g + h
    if n is goal:
        return path
    for each neighbor of n:
        if neighbor is occupied or inflated:
            continue
        tentative_g = g[n] + movement_cost
        if tentative_g < g[neighbor]:
            parent[neighbor] = n
            g[neighbor] = tentative_g
            add neighbor to open_set
```

Heuristic:
- h = Euclidean distance to goal

---

### Path Smoothing (Pseudocode)
```text
for i from 0 to path_length:
    for j from end to i:
        if line_of_sight(path[i], path[j]):
            remove intermediate points
```

---

## Configuration
All simulation parameters are defined in `config.py`, including:
- Map size and grid resolution
- LIDAR range and number of beams
- Log-odds update values
- Safety distance for planning
- Simulation time step

---

## Evaluation Metrics
- Mapping accuracy: IoU, Precision, Recall
- Planning performance: success rate, planning time, path length
- Trajectory accuracy: Absolute Trajectory Error (ATE)

---

## Limitations
- Robot pose assumed known (no localization filter)
- Idealized LIDAR model
- Static environment
- No ROS / ROS2 integration

---

## References
- Thrun et al., Probabilistic Robotics
- Elfes, Occupancy Grid Mapping
- Hart et al., A* Search Algorithm

---

## License
MIT License
