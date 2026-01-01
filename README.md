# Autonomous-Mobile-Robot-SLAM-Navigation-on-2D-Environmet
An integrated Python-based robotics simulation environment featuring occupancy grid mapping, Lidar raycasting, A path planning*, and real-time telemetry. This project demonstrates a full-stack robotics pipeline, from raw sensor processing to high-level motion planning.

Notation and coordinate frames

World frame: Cartesian coordinates 
(
𝑥
,
𝑦
)
(x,y) in meters. Robot pose at time 
𝑡
t: 
𝑥
𝑡
=
[
𝑥
𝑡
,
  
𝑦
𝑡
,
  
𝜃
𝑡
]
⊤
x
t
	​

=[x
t
	​

,y
t
	​

,θ
t
	​

]
⊤
, where 
𝜃
θ is yaw (radians).

Grid: occupancy grid with 
𝑁
𝑥
×
𝑁
𝑦
N
x
	​

×N
y
	​

 cells. Cell indices 
(
𝑖
,
𝑗
)
(i,j) map to world coordinates via

𝑥
=
𝑥
min
⁡
+
(
𝑖
+
1
2
)
⋅
Δ
,
𝑦
=
𝑦
min
⁡
+
(
𝑗
+
1
2
)
⋅
Δ
x=x
min
	​

+(i+
2
1
	​

)⋅Δ,y=y
min
	​

+(j+
2
1
	​

)⋅Δ

where 
Δ
Δ is the cell size (meters).

LIDAR beams: 
𝑏
=
1
…
𝐵
b=1…B. A beam 
𝑏
b from robot at pose 
𝑥
𝑡
x
t
	​

 has direction 
𝜙
𝑏
ϕ
b
	​

 (robot frame) and reports range 
𝑧
𝑡
,
𝑏
z
t,b
	​

.

3. Robot kinematics (motion model)

The simulation uses the unicycle (nonholonomic) kinematic model. Control inputs are linear velocity 
𝑣
𝑡
v
t
	​

 and angular velocity 
𝜔
𝑡
ω
t
	​

.

Continuous-time:

𝑥
˙
=
𝑣
cos
⁡
𝜃
,
𝑦
˙
=
𝑣
sin
⁡
𝜃
,
𝜃
˙
=
𝜔
.
x
˙
=vcosθ,
y
˙
	​

=vsinθ,
θ
˙
=ω.

Discrete-time (Euler integration, time-step 
Δ
𝑡
Δt):

𝑥
𝑡
+
Δ
𝑡
=
𝑥
𝑡
+
𝑣
𝑡
cos
⁡
(
𝜃
𝑡
)
 
Δ
𝑡
+
𝜖
𝑥
,
𝑦
𝑡
+
Δ
𝑡
=
𝑦
𝑡
+
𝑣
𝑡
sin
⁡
(
𝜃
𝑡
)
 
Δ
𝑡
+
𝜖
𝑦
,
x
t+Δt
	​

=x
t
	​

+v
t
	​

cos(θ
t
	​

)Δt+ϵ
x
	​

,y
t+Δt
	​

=y
t
	​

+v
t
	​

sin(θ
t
	​

)Δt+ϵ
y
	​

,
𝜃
𝑡
+
Δ
𝑡
=
𝜃
𝑡
+
𝜔
𝑡
 
Δ
𝑡
+
𝜖
𝜃
,
θ
t+Δt
	​

=θ
t
	​

+ω
t
	​

Δt+ϵ
θ
	​

,

where 
𝜖
𝑥
,
𝜖
𝑦
,
𝜖
𝜃
ϵ
x
	​

,ϵ
y
	​

,ϵ
θ
	​

 are odometry noise terms (typically zero-mean Gaussian: 
𝑁
(
0
,
𝜎
2
)
N(0,σ
2
)). Use additive noise to simulate wheel slip and sensor imperfections. For experiments, report the noise standard deviations used.

4. LIDAR sensor model (measurement model)

Each LIDAR beam measures a range 
𝑧
z along direction 
𝜙
ϕ (in robot frame). The simple sensor model used by the simulation:

Measurement generation:

𝑧
=
min
⁡
(
𝑧
true
+
𝜂
,
  
𝑧
max
⁡
)
,
𝜂
∼
𝑁
(
0
,
𝜎
𝑧
2
)
,
z=min(z
true
	​

+η,z
max
	​

),η∼N(0,σ
z
2
	​

),

where 
𝑧
true
z
true
	​

 is the true distance from robot to the first obstacle along the beam, and 
𝑧
max
⁡
z
max
	​

 is sensor range limit.

Practical implementation uses raycasting against the ground-truth floorplan image: traverse along ray until obstacle pixel encountered or 
𝑧
max
⁡
z
max
	​

 reached. The simulator optionally discretizes the ray into steps 
Δ
𝑠
Δ
s
	​

 or uses Bresenham / DDA to derive exact cell intersections.

5. Occupancy grid mapping — log-odds formulation

We use a Bayesian occupancy grid represented in log-odds form (numerically stable and incremental).

Define cell occupancy probability at time 
𝑡
t: 
𝑝
𝑡
=
𝑃
(
occupied
∣
𝑧
1
:
𝑡
)
p
t
	​

=P(occupied∣z
1:t
	​

). The log-odds is:

𝐿
𝑡
=
log
⁡
𝑝
𝑡
1
−
𝑝
𝑡
.
L
t
	​

=log
1−p
t
	​

p
t
	​

	​

.

Recursive update with inverse sensor model 
𝑙
(
⋅
)
l(⋅):

𝐿
𝑡
(
𝑚
)
=
𝐿
𝑡
−
1
(
𝑚
)
+
𝑙
(
𝑚
;
𝑧
𝑡
,
𝑥
𝑡
)
−
𝐿
0
,
L
t
	​

(m)=L
t−1
	​

(m)+l(m;z
t
	​

,x
t
	​

)−L
0
	​

,

where:

𝑚
m denotes a grid cell,

𝐿
0
=
log
⁡
𝑝
0
1
−
𝑝
0
L
0
	​

=log
1−p
0
	​

p
0
	​

	​

 is prior log-odds (commonly 
𝑝
0
=
0.5
⇒
𝐿
0
=
0
p
0
	​

=0.5⇒L
0
	​

=0),

𝑙
(
𝑚
;
𝑧
𝑡
,
𝑥
𝑡
)
l(m;z
t
	​

,x
t
	​

) is the inverse sensor model's log-odds evidence contributed by the current measurement 
𝑧
𝑡
z
t
	​

.

Simplified practical implementation:

For cells along the ray (free space): add 
𝐿
free
<
0
L
free
	​

<0.

For the hit cell at measured range (occupied): add 
𝐿
occ
>
0
L
occ
	​

>0.

Clamp 
𝐿
𝑡
(
𝑚
)
L
t
	​

(m) to 
[
𝐿
min
⁡
,
𝐿
max
⁡
]
[L
min
	​

,L
max
	​

] to avoid numerical explosion. Convert to probability for visualization:

𝑝
𝑡
(
𝑚
)
=
1
1
+
exp
⁡
(
−
𝐿
𝑡
(
𝑚
)
)
.
p
t
	​

(m)=
1+exp(−L
t
	​

(m))
1
	​

.

Implementation notes

Use Bresenham’s algorithm to compute discrete cells along the beam. Complexity per beam: 
𝑂
(
𝑘
)
O(k) where 
𝑘
k is number of traversed cells.

Use integer arithmetic for Bresenham to avoid bias.

Typical choices: 
𝐿
occ
∈
[
0.85
,
2.0
]
L
occ
	​

∈[0.85,2.0], 
𝐿
free
∈
[
−
0.4
,
−
1.0
]
L
free
	​

∈[−0.4,−1.0], bounds 
𝐿
min
⁡
≈
−
4
,
  
𝐿
max
⁡
≈
4
L
min
	​

≈−4,L
max
	​

≈4. (Tune per experiment.)

6. Ray traversal: Bresenham vs continuous stepping

Two common approaches:

Bresenham (grid-cell exact traversal)

Input: ray endpoints in grid index space.

Output: ordered list of cells intersected by the ray.

Pros: exact integer traversal; efficient; correct for occupancy updates.

Complexity: 
𝑂
(
𝑘
)
O(k) for 
𝑘
k cells.

Fixed-step ray marching

Step along ray in small increments 
Δ
𝑠
Δs, convert positions to cell indices.

Pros: easy to implement and extend to subcell effects.

Cons: slower for long ranges, potential redundancy in cell visits.

Prefer Bresenham for the occupancy grid mapping module; ray-marching is acceptable for visualization or continuous maps.

7. Planner: A* on occupancy grid

The planner works in grid cell coordinates. Let nodes be cells 
𝑛
n. A* maintains:

𝑔
(
𝑛
)
g(n): cost from start to node 
𝑛
n,

ℎ
(
𝑛
)
h(n): heuristic estimate (admissible),

𝑓
(
𝑛
)
=
𝑔
(
𝑛
)
+
ℎ
(
𝑛
)
f(n)=g(n)+h(n).

Transition model:

Use 8-connected neighbors 
𝑁
8
(
𝑛
)
N
8
	​

(n).

Move cost from 
𝑛
n to 
𝑚
m:

𝑐
(
𝑛
,
𝑚
)
=
{
inf
	
if 
𝑚
 is occupied or inside inflation radius


𝑑
(
𝑛
,
𝑚
)
	
otherwise
c(n,m)={
inf
d(n,m)
	​

if m is occupied or inside inflation radius
otherwise
	​


where 
𝑑
(
𝑛
,
𝑚
)
d(n,m) is Euclidean distance between cell centers (1 or 
2
2
	​

).

Heuristic:

Use Euclidean (or Chebyshev) distance: admissible and consistent:

ℎ
(
𝑛
)
=
𝛼
⋅
∥
 
𝑥
𝑛
−
𝑥
goal
 
∥
2
,
h(n)=α⋅∥x
n
	​

−x
goal
	​

∥
2
	​

,

with 
𝛼
=
1
α=1.

Obstacle inflation:

Inflate obstacles by a safety radius 
𝑟
safe
r
safe
	​

 (in meters) converted to cells 
𝑅
cells
=
⌈
𝑟
safe
/
Δ
⌉
R
cells
	​

=⌈r
safe
	​

/Δ⌉.

Practically performed by morphological dilation of binary occupied mask.

Path smoothing:

Shortcutting: iterate over path points, attempt to replace sequences with direct line-of-sight segments (use Bresenham collision check). This reduces number of waypoints and jaggedness.

Spline fitting: optionally fit cubic splines to waypoints to generate smooth curvature-continuous trajectories; ensure the spline stays within safe clearance using sampling-based collision checks.

Complexity:

A* average-case: 
𝑂
(
𝑛
log
⁡
𝑛
)
O(nlogn) with a binary heap and reasonable heuristic; worst-case explores much of grid.

8. Odometry & localization (basic)

This simulation assumes pose estimates are available (perfect or noisy odometry). Two operating modes:

Ground-truth pose: use the exact simulated pose (no localization filter). Useful to evaluate mapping quality with perfect localization.

Noisy odometry: integrate commanded controls with additive noise. Use this to test robustness of mapping and planner. If desired, extend with EKF / particle filter for localization.

9. Evaluation metrics (mapping & planning)
Mapping

Intersection over Union (IoU) of reconstructed binary occupancy map vs ground truth:

IoU
=
∣
𝑀
est
∩
𝑀
gt
∣
∣
𝑀
est
∪
𝑀
gt
∣
IoU=
∣M
est
	​

∪M
gt
	​

∣
∣M
est
	​

∩M
gt
	​

∣
	​


Precision / Recall of occupied cell classification.

Log-likelihood or KL divergence of map probability distributions (for probabilistic analysis).

Trajectory (if robot follows planned path)

Absolute Trajectory Error (ATE): RMS positional error between executed trajectory and reference.

Relative Pose Error (RPE): local drift per time interval.

Planner

Success rate (fraction of start/goal pairs where a valid path was found).

Planning time (ms).

Path length and clearance statistics.

Number of replans and replan frequency under dynamic obstacles or mapping changes.

10. Implementation / reproducibility details

Use consistent units (meters, seconds). Document MAP_SIZE_METERS, CELL_SIZE, LIDAR_RANGE, LIDAR_BEAMS, and noise parameters in config.py.

For reproducible results, set random seeds for sensor noise and any randomized planner components.

Record all hyperparameters used in each experiment and include them in experiment logs (JSON or YAML).

For each experiment report: map image used, grid resolution, LIDAR beams & noise, log-odds parameters, planner inflation radius, smoothing method.

11. Typical default parameters (examples)

These are recommendations; use config.py to set exact values.

Map: MAP_SIZE_METERS = 20.0, CELL_SIZE = 0.05 (400 × 400 grid).

LIDAR: LIDAR_RANGE = 8.0 (m), LIDAR_BEAMS = 180, σ_z = 0.02 (m).

Log-odds: L_OCC = +0.9, L_FREE = -0.4, L_MAX = +4, L_MIN = -4.

Planner: r_safe = 0.25 (m) → inflation radius R_cells = ceil(0.25/CELL_SIZE).

12. Numerical stability and pitfalls (important for academic write-ups)

Clamping log-odds prevents overflow and overconfidence; report bounds used.

Sensor model mismatch: unrealistic inverse sensor model (too-strong L_OCC) can lead to permanent false positives — validate using IoU and precision/recall.

Grid aliasing: coarse grids produce aliasing in mapping and planning; always report cell size.

Edge-case beams: beams that graze corners may produce inconsistent hits due to discretization — document ray resolution.

13. Suggested experiments & ablation studies (for thesis chapters)

Resolution sweep: fix the physical map; vary CELL_SIZE and compute IoU and mapping time vs grid size.

Beam density sweep: vary LIDAR_BEAMS (e.g., 36, 90, 180, 360) and report mapping convergence speed and CPU time.

Log-odds sensitivity: grid search over L_OCC and L_FREE and report final IoU and false positive rate.

Noisy odometry: vary odometry noise levels; evaluate mapping robustness with perfect vs noisy pose.

Planning under uncertainty: measure replanning frequency when map is incrementally updated online and obstacles appear/disappear.

Comparison: implement both Bresenham and fixed-step raycasting and report runtime and mapping accuracy.

14. References (short list for academic README)

Elfes, A. “Occupancy Grids: A Probabilistic Framework for Robot Perception and Navigation.” (Foundational concept for occupancy grids.)

Hart, P., Nilsson, N., Raphael, B. “A Formal Basis for the Heuristic Determination of Minimum Cost Paths.” (A* algorithm.)

Bresenham, J. “Algorithm for computer control of a digital plotter.” (Bresenham’s algorithm for raster traversal.)
