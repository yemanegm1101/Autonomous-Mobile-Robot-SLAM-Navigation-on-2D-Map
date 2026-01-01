import numpy as np
import math
from config import *

class Lidar:
    def __init__(self):
        self.angles = np.linspace(0, 2*math.pi, LIDAR_BEAMS, endpoint=False)

    def scan(self, robot_pose, truth_map):
        """
        Raycasting against a Grid Map (Truth Map).
        truth_map: 2D numpy array where > 0 is obstacle.
        """
        hits = []
        rx, ry, r_theta = robot_pose
        
        # Grid dimensions from map
        h, w = truth_map.shape
        
        # Conversion factors
        # Assuming truth_map covers MAP_SIZE_METERS x MAP_SIZE_METERS
        pixels_per_meter_x = w / MAP_SIZE_METERS
        pixels_per_meter_y = h / MAP_SIZE_METERS
        
        # Robot position in map grid coordinates
        rx_grid = rx * pixels_per_meter_x
        ry_grid = ry * pixels_per_meter_y
        
        for angle in self.angles:
            global_angle = r_theta + angle
            
            sin_a = math.sin(global_angle)
            cos_a = math.cos(global_angle)
            
            # fast voxel traversal (similar to DDA) could be used, 
            # or simple stepping for simplicity. 
            # Let's use simple stepping for readability and generic support,
            # though less efficient than DDA. 
            # Step size = 1 pixel roughly?
            
            step_size = 0.5 # Half a cell/pixel size for precision
            max_steps = int((LIDAR_RANGE * pixels_per_meter_x) / step_size)
            
            hit_found = False
            for i in range(max_steps):
                dist_px = i * step_size
                
                check_x = int(rx_grid + dist_px * cos_a)
                check_y = int(ry_grid + dist_px * sin_a)
                
                if check_x < 0 or check_x >= w or check_y < 0 or check_y >= h:
                    break # Out of map bounds
                
                # Check collision (threshold could be 0.5 or 127 depending on map loading)
                # Assuming truth_map is 0-255 or 0-1. Let's say > 0 is obstacle.
                if truth_map[check_y, check_x] > 0:
                    # Hit!
                    # Convert back to meters
                    hx_m = check_x / pixels_per_meter_x
                    hy_m = check_y / pixels_per_meter_y
                    hits.append((hx_m, hy_m))
                    hit_found = True
                    break
            
            # If no hit within max range, we don't return a point (as per previous logic)
                
        return hits
