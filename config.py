import math
# config.py
MAP_SIZE_METERS = 20.0
GRID_CELLS = 100
CELL_SIZE = MAP_SIZE_METERS / GRID_CELLS
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 700
SCREEN_SIZE = (SCREEN_WIDTH, SCREEN_HEIGHT) # For backwards compatibility if needed, though tuple is better

ROBOT_Radius = 0.3
LIDAR_RANGE = 8.0
LIDAR_BEAMS = 180

# Mapping Constants (Log Odds)
L_OCC = 0.8
L_FREE = -0.4
L_MAX = 5.0
L_MIN = -5.0

# Collision Avoidance
SAFETY_DISTANCE = 0.6  # Meters
COLLISION_FOV = math.pi / 3  # 60 degrees in front
