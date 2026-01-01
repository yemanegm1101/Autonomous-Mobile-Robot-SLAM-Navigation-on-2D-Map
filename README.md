# 2D SLAM and Path Planning Simulation

This repository implements a compact **2D robotic navigation simulation** that combines:
- Simulated LIDAR sensing
- Log-odds occupancy grid mapping
- A* path planning with obstacle inflation
- Path smoothing using line-of-sight checks
- Unicycle kinematic robot model

The project is intended for **educational use, research simulation, and robotics education**.

---

## Semantic Features to Consider for SLAM & Planning

The user-provided floor map (Floormap.png) is treated by **mapper.py** as the ground-truth occupancy image: dark strokes correspond to walls/obstacles and white areas to free space.**mapper.py** loads and binarizes the image (grayscale thresholding, optional morphological closing/thinning), converts pixels to grid cells using the pixel-to-meter scale defined in **config.py**, and produces a boolean occupancy grid used for LIDAR raycasting and metric evaluation. Note that wall thickness in the image becomes blocks of occupied cells (so either thin the walls or account for them with the planner’s inflation radius); the mapper saves a cleaned binary PNG and a occupancy array that the simulation and planner consume as the environment truth.

<img width="1200" height="1166" alt="Floormap" src="https://github.com/user-attachments/assets/cdefc68c-ad29-44e0-91fd-6e0a68d13713" />


The video presents a real-time demonstration of a 2D robotic SLAM (Simultaneous Localization and Mapping) simulation. A green robot acts as the central agent, navigating through an initially dark, unexplored environment while emitting a fan of gray Lidar rays to scan its surroundings. As these rays contact invisible obstacles, the system dynamically constructs an occupancy grid map, revealing the floorplan by marking free space in white and detected walls in blue. The simulation cease in an autonomous navigation sequence where a cyan line appears, visualizing the optimal path calculated by the **A*** algorithm, which the robot smoothly follows using Unicycle kinematic robot model to reach its destination while avoiding collisions.




https://github.com/user-attachments/assets/4258d3df-d660-4cb5-9524-5cca4294cca4


---

## System Pipeline
1. Robot motion update using kinematic equations  
2. LIDAR raycasting against a ground-truth map  
3. Occupancy grid update using log-odds  
4. A* path planning on the grid  
5. Path smoothing  
6. Visualization and telemetry  

---
## Coordinate Frames and Notation

<img width="778" height="254" alt="image" src="https://github.com/user-attachments/assets/bd08ff3f-672c-4c08-a411-69331e26fd95" />

---

## Mathematical Models 

### Robot Motion Model
The simulation uses the unicycle (nonholonomic) kinematic model. Control inputs are linear velocity 𝑣𝑡 and angular velocity 𝜔𝑡.

Motion equations:

<img width="324" height="38" alt="image" src="https://github.com/user-attachments/assets/3e71b557-216f-485a-ac4e-1fd5b3a7be5c" />

Discrete-time update:

<img width="266" height="114" alt="image" src="https://github.com/user-attachments/assets/04433e88-98f4-449e-8903-f210be5d0db1" />


Optional Gaussian noise may be added to simulate odometry uncertainty.

---

### LIDAR Measurement Model
The LIDAR sensor emits 𝐵 beams over a fixed angular range.

<img width="454" height="44" alt="image" src="https://github.com/user-attachments/assets/6ec94355-ed6f-4914-bd55-3a81c4a12865" />


Raycasting is used to detect the first obstacle along each beam.

---

### Occupancy Grid Mapping (Log-Odds)
Each grid cell stores a log-odds value:

<img width="315" height="75" alt="image" src="https://github.com/user-attachments/assets/6a097dd6-2267-4db4-bf6d-766e58120deb" />


Recursive update:

<img width="313" height="56" alt="image" src="https://github.com/user-attachments/assets/6ba4f0ab-9fc1-4bfe-8d31-26a07d0d9280" />


Inverse sensor model:
- Cells along a beam before obstacle:
  
<img width="185" height="38" alt="image" src="https://github.com/user-attachments/assets/03fe1ac0-27c4-4648-b58b-e7970b0bd044" />

- Cell where obstacle is detected:
  
<img width="152" height="36" alt="image" src="https://github.com/user-attachments/assets/42675f90-7482-4a8d-848f-ad449b8faba3" />


Log-odds values are clamped:

  <img width="171" height="35" alt="image" src="https://github.com/user-attachments/assets/e2e3a2d4-5260-4214-a16e-1068beecf590" />

Occupancy probability (for visualization):

<img width="193" height="61" alt="image" src="https://github.com/user-attachments/assets/57a2e4d1-400c-42ba-91f4-63cda8f3d717" />

Ray traversal uses Bresenham’s algorithm for accurate grid cell updates.

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

Path planning is performed on the occupancy grid using **A***.

<img width="650" height="156" alt="image" src="https://github.com/user-attachments/assets/f7221017-d10a-427d-b524-80b44158a4b4" />

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
- Elfes, A., Occupancy Grids: A Probabilistic Framework for Robot Perception and Navigation

- Hart et al., A Formal Basis for the Heuristic Determination of Minimum Cost Paths

- Thrun et al., Probabilistic Robotics

---

## License
MIT License
