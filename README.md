# 2D SLAM and Path Planning Simulation

## Overview
This repository implements a compact **2D robotic navigation simulation** integrating:
- Simulated LIDAR sensing
- Log-odds occupancy grid mapping
- A* path planning with obstacle inflation
- Path smoothing via line-of-sight checks
- Unicycle kinematic robot model

The project is designed for robotics education.

---

## System Pipeline
1. Robot motion update (kinematics)
2. LIDAR raycasting
3. Occupancy grid update (log-odds)
4. A* path planning
5. Path smoothing
6. Visualization

---

## Mathematical Models

### Robot Motion (Unicycle)
\[
\dot{x} = v\cos\theta, \quad
\dot{y} = v\sin\theta, \quad
\dot{\theta} = \omega
\]

Discrete update:
\[
x_{t+1} = x_t + v_t\cos(\theta_t)\Delta t, \quad
y_{t+1} = y_t + v_t\sin(\theta_t)\Delta t
\]

---

### LIDAR Measurement Model
\[
z = \min(z_{true} + \eta, z_{max}), \quad \eta \sim \mathcal{N}(0,\sigma^2)
\]

---

### Occupancy Grid (Log-Odds)
\[
L = \log\frac{P(occ)}{1-P(occ)}
\]

Update rule:
\[
L_t(c) = L_{t-1}(c) + l(c|z_t, x_t)
\]

---

## Core Algorithms

### Occupancy Grid Update (Pseudocode)
```text
for each lidar beam:
    trace cells using Bresenham
    for each cell before hit:
        L[cell] += L_free
    if obstacle detected:
        L[hit_cell] += L_occ
    clamp L[cell] to [L_min, L_max]
```

---

### A* Path Planning (Pseudocode)
```text
open_set ← {start}
while open_set not empty:
    n ← node with lowest f = g + h
    if n == goal:
        return reconstruct_path()
    for each neighbor of n:
        if neighbor is occupied:
            continue
        tentative_g ← g[n] + cost(n, neighbor)
        if tentative_g < g[neighbor]:
            parent[neighbor] ← n
            g[neighbor] ← tentative_g
            add neighbor to open_set
```

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
All parameters are defined in `config.py`:
- Map size and resolution
- LIDAR range and beam count
- Log-odds values
- Safety distance
- Simulation timestep

---

## Evaluation Metrics
- Mapping IoU, Precision, Recall
- Planning time and success rate
- Path length
- Trajectory error (ATE)

---

## Limitations
- Known robot pose (no localization filter)
- Idealized LIDAR
- Static environment
- No ROS integration

---

## References
- Thrun et al., *Probabilistic Robotics*
- Elfes, *Occupancy Grid Mapping*
- Hart et al., *A* Search Algorithm*

---

## License
MIT License
