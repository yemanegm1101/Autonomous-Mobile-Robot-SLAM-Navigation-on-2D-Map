import heapq
import math
import numpy as np
from config import *

class AStarPlanner:
    def __init__(self, occupancy_grid):
        self.grid_obj = occupancy_grid

    def plan(self, start_pose, goal_pose):
        start_node = self.grid_obj.world_to_grid(start_pose[0], start_pose[1])
        end_node = self.grid_obj.world_to_grid(goal_pose[0], goal_pose[1])
        
        # Check bounds
        if not (0 <= start_node[0] < GRID_CELLS and 0 <= start_node[1] < GRID_CELLS) or \
           not (0 <= end_node[0] < GRID_CELLS and 0 <= end_node[1] < GRID_CELLS):
            return []

        # Priority Queue: (f_score, gx, gy)
        open_list = []
        heapq.heappush(open_list, (0, start_node[0], start_node[1]))
        
        came_from = {}
        g_score = {start_node: 0}
        
        while open_list:
            current_f, cx, cy = heapq.heappop(open_list)
            current = (cx, cy)
            
            if current == end_node:
                return self.reconstruct_path(came_from, current)

            # 8-Connected Neighbors (Up, Down, Left, Right + Diagonals)
            neighbors = [
                (0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0),
                (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
            ]
            
            for dx, dy, cost in neighbors:
                nx, ny = cx + dx, cy + dy
                neighbor = (nx, ny)
                
                # Check bounds
                if not (0 <= nx < GRID_CELLS and 0 <= ny < GRID_CELLS):
                    continue
                    
                # Check obstacle (Log Odds > 0 means occupied)
                # Inflation check: check neighbors for obstacles
                is_occupied = False
                inflation_radius = 2 # cells (~6cm if grid is 100x100 for 20m)
                # Actually, 20m / 100 = 0.2m (20cm) per cell.
                # So 2 cells = 40cm. SAFETY_DISTANCE is 0.6m (3 cells).
                # Let's use 3 cells inflation.
                
                check_radius = 2 
                for iy in range(-check_radius, check_radius + 1):
                    for ix in range(-check_radius, check_radius + 1):
                        ny_inf, nx_inf = ny + iy, nx + ix
                        if 0 <= nx_inf < GRID_CELLS and 0 <= ny_inf < GRID_CELLS:
                            if self.grid_obj.grid[ny_inf, nx_inf] > 0:
                                is_occupied = True
                                break
                    if is_occupied: break
                
                if is_occupied:
                    continue
                
                # Use move cost (1.0 or 1.414)
                tentative_g = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, end_node)
                    heapq.heappush(open_list, (f_score, nx, ny))
                    
        return [] # No path found


    def heuristic(self, a, b):
        # Euclidean distance
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        
        # Smooth path
        path = self.smooth_path(path)
        
        # Convert grid coords back to world coords
        world_path = []
        for gx, gy in path:
            wx = (gx * CELL_SIZE) + (CELL_SIZE / 2) # Center of cell
            wy = (gy * CELL_SIZE) + (CELL_SIZE / 2)
            world_path.append((wx, wy))
            
        return world_path

    def smooth_path(self, path):
        if len(path) < 3:
            return path
            
        smoothed = [path[0]]
        current_idx = 0
        
        while current_idx < len(path) - 1:
            # Look ahead as far as possible
            last_valid = current_idx + 1
            for i in range(current_idx + 2, len(path)):
                if self.is_line_clear(path[current_idx], path[i]):
                    last_valid = i
            
            smoothed.append(path[last_valid])
            current_idx = last_valid
            
        return smoothed

    def is_line_clear(self, p1, p2):
        # Raycast/Bresenham check between grid cells p1 and p2
        # Returns True if clear of obstacles (and safe margin)
        x0, y0 = p1
        x1, y1 = p2
        
        points = self.bresenham_line(x0, y0, x1, y1)
        for (x, y) in points:
             # Check obstacle
             if not (0 <= x < GRID_CELLS and 0 <= y < GRID_CELLS):
                 return False
             if self.grid_obj.grid[y, x] > 0:
                 return False
             
        # Check safety margin (check neighbors for obstacles)
        # Using a slightly larger radius for smoothing to ensure we don't cut corners too close
        check_radius = 3 
        for (x, y) in points:
             if not (0 <= x < GRID_CELLS and 0 <= y < GRID_CELLS):
                 return False
             
             for dy in range(-check_radius, check_radius + 1):
                 for dx in range(-check_radius, check_radius + 1):
                     nx, ny = x + dx, y + dy
                     if 0 <= nx < GRID_CELLS and 0 <= ny < GRID_CELLS:
                         if self.grid_obj.grid[ny, nx] > 0:
                             return False
        return True

    def bresenham_line(self, x0, y0, x1, y1):
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
