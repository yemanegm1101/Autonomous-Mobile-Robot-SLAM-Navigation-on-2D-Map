import numpy as np
from config import *

class OccupancyGrid:
    def __init__(self):
        # 2D Array initialized to 0 (Unknown)
        # grid[y, x] where y is row (vertical), x is col (horizontal)
        self.grid = np.zeros((GRID_CELLS, GRID_CELLS))

    def world_to_grid(self, x, y):
        gx = int(x / CELL_SIZE)
        gy = int(y / CELL_SIZE)
        return gx, gy

    def bresenham(self, x0, y0, x1, y1):
        """
        Bresenham's Line Algorithm
        Returns list of (x, y) coordinates
        """
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        
        x, y = x0, y0
        
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
                
        points.append((x, y))
        return points

    def update(self, robot_pose, scan_points):
        """
        Inverse Sensor Model
        1. Raytrace from robot to hit point.
        2. Free space along the ray (subtract log odds).
        3. Occupied space at the hit point (add log odds).
        """
        start_gx, start_gy = self.world_to_grid(robot_pose[0], robot_pose[1])
        
        # Ensure start is within grid
        if not (0 <= start_gx < GRID_CELLS and 0 <= start_gy < GRID_CELLS):
            return

        for point in scan_points:
            end_gx, end_gy = self.world_to_grid(point[0], point[1])
            
            # Get all cells the laser passed through
            cells = self.bresenham(start_gx, start_gy, end_gx, end_gy)
            
            # Update Free Space (all except last)
            for (cx, cy) in cells[:-1]: 
                if 0 <= cx < GRID_CELLS and 0 <= cy < GRID_CELLS:
                    self.grid[cy, cx] += L_FREE
                    self.grid[cy, cx] = max(self.grid[cy, cx], L_MIN)

            # Update Occupied Space (the last point)
            if 0 <= end_gx < GRID_CELLS and 0 <= end_gy < GRID_CELLS:
                self.grid[end_gy, end_gx] += L_OCC
                self.grid[end_gy, end_gx] = min(self.grid[end_gy, end_gx], L_MAX)
