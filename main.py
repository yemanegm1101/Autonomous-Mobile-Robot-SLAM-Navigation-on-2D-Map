import pygame
import numpy as np
import sys
import math
from config import *
from robot import Robot
from lidar import Lidar
from mapper import OccupancyGrid
from planning import AStarPlanner
import multiprocessing
from telemetry import start_telemetry

# Camera Class for Pan/Zoom handling
class Camera:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.zoom = 1.0
        self.offset_x = 0
        self.offset_y = 0
        self.dragging = False
        self.last_mouse_pos = (0, 0)
    
    def world_to_screen(self, wx, wy):
        # Convert meters to pixels first (base scale)
        # Then apply camera zoom and offset
        
        # Scale to fit 85% of the screen height
        base_scale = (self.height * 0.85) / MAP_SIZE_METERS
        
        # Center the map: (ScreenW - MapW)/2, (ScreenH - MapH)/2
        map_screen_w = MAP_SIZE_METERS * base_scale * self.zoom
        map_screen_h = MAP_SIZE_METERS * base_scale * self.zoom
        
        center_offset_x = (self.width - map_screen_w) / 2
        center_offset_y = (self.height - map_screen_h) / 2
        
        sx = (wx * base_scale * self.zoom) + self.offset_x + center_offset_x
        sy = (wy * base_scale * self.zoom) + self.offset_y + center_offset_y
        return int(sx), int(sy)
    
    def screen_to_world(self, sx, sy):
        base_scale = (self.height * 0.85) / MAP_SIZE_METERS
        
        map_screen_w = MAP_SIZE_METERS * base_scale * self.zoom
        map_screen_h = MAP_SIZE_METERS * base_scale * self.zoom
        
        center_offset_x = (self.width - map_screen_w) / 2
        center_offset_y = (self.height - map_screen_h) / 2

        wx = (sx - self.offset_x - center_offset_x) / (base_scale * self.zoom)
        wy = (sy - self.offset_y - center_offset_y) / (base_scale * self.zoom)
        return wx, wy
    
    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 4: # Wheel Up
                self.zoom *= 1.1
            elif event.button == 5: # Wheel Down
                self.zoom /= 1.1
            elif event.button == 3: # Right click for pan
                self.dragging = True
                self.last_mouse_pos = event.pos
                
        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 3:
                self.dragging = False
                
        elif event.type == pygame.MOUSEMOTION:
            if self.dragging:
                dx = event.pos[0] - self.last_mouse_pos[0]
                dy = event.pos[1] - self.last_mouse_pos[1]
                self.offset_x += dx
                self.offset_y += dy
                self.last_mouse_pos = event.pos
        
        elif event.type == pygame.VIDEORESIZE:
            self.width = event.w
            self.height = event.h


def load_map_image(filename):
    try:
        # Load image
        img = pygame.image.load(filename)
        # Convert to grayscale 2D array
        # This will be used as Truth Map
        # 0 = Free, 1 = Occ
        
        width, height = img.get_size()
        
        arr = pygame.surfarray.array3d(img)
        gray = np.mean(arr, axis=2)
        
        truth_map = np.zeros((height, width), dtype=np.uint8)
        truth_map = (gray.T < 128).astype(np.uint8)
        
        return truth_map
    except Exception as e:
        print(f"Failed to load map: {e}")
        m = np.zeros((500, 500), dtype=np.uint8)
        m[0,:] = 1; m[-1,:] = 1; m[:,0] = 1; m[:,-1] = 1
        return m

def check_collision(robot_pose, scan_points):
    """Checks if any lidar points are within SAFETY_DISTANCE and in front of the robot"""
    for p in scan_points:
        dx = p[0] - robot_pose[0]
        dy = p[1] - robot_pose[1]
        dist = math.hypot(dx, dy)
        
        if dist < SAFETY_DISTANCE:
            # Check if point is in front
            angle_to_point = math.atan2(dy, dx)
            angle_diff = angle_to_point - robot_pose[2]
            # Normalize to [-pi, pi]
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
            
            if abs(angle_diff) < COLLISION_FOV / 2:
                return True
    return False

def main():
    pygame.init()
    
    # Resizable window
    screen = pygame.display.set_mode(SCREEN_SIZE, pygame.RESIZABLE)
    pygame.display.set_caption("SLAM Simulation")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Consolas", 20)
    
    # Load Truth Map
    truth_map = load_map_image("Floormap.png")
    
    # 0. Initialize Telemetry
    telemetry_queue = multiprocessing.Queue()
    telemetry_process = multiprocessing.Process(target=start_telemetry, args=(telemetry_queue,))
    telemetry_process.start()
    
    # 1. Initialize Objects
    robot = Robot(x=10, y=10, theta=0)
    lidar = Lidar()
    mapper = OccupancyGrid()
    planner = AStarPlanner(mapper)
    camera = Camera(SCREEN_WIDTH, SCREEN_HEIGHT) # Initial size
    
    path = []
    final_goal = None
    scan_points = []
    replan_cooldown = 0
    planning_status = 0.0 # 1.0: Success, -1.0: Error, 0.0: Idle
    running = True

    while running:
        dt = clock.tick(30) / 1000.0  # limit to 30fps

        # 2. Handle Inputs
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            # Pass events to camera
            camera.handle_event(event) # handles resize etc.
            if event.type == pygame.VIDEORESIZE:
                # Update display surface
                screen = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)
            
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1: # Left click
                    # Set goal and Plan Path
                    mx, my = pygame.mouse.get_pos()
                    # Convert screen to world
                    wx, wy = camera.screen_to_world(mx, my)
                    
                    print(f"Planning path to {wx:.2f}, {wy:.2f}")
                    final_goal = (wx, wy)
                    # Send segment marker to telemetry
                    telemetry_queue.put({"SEGMENT": f"Goal: ({wx:.1f}, {wy:.1f})"})
                    
                    path = planner.plan(robot.pose, final_goal)
                    if not path:
                        print("No path found!")
                        planning_status = -1.0
                    else:
                        print(f"Path found with {len(path)} nodes")
                        planning_status = 1.0

        keys = pygame.key.get_pressed()
        robot.velocity = 0.0
        robot.omega = 0.0
        
        if keys[pygame.K_UP]: robot.velocity = 2.0
        if keys[pygame.K_DOWN]: robot.velocity = -2.0
        if keys[pygame.K_LEFT]: robot.omega = -2.0 
        if keys[pygame.K_RIGHT]: robot.omega = 2.0
        
        # Autonomous Path Following
        if path:
            # Look ahead logic
            target = path[0] # Target the next (or first) node
            dx = target[0] - robot.pose[0]
            dy = target[1] - robot.pose[1]
            dist = math.hypot(dx, dy)
            
            # Threshold to reach point
            if dist < 0.2: # 20cm tolerance
                path.pop(0) # Reached, remove
                if not path:
                    planning_status = 0.0 # Goal reached
            else:
                # Calculate heading
                target_heading = math.atan2(dy, dx)
                heading_err = target_heading - robot.pose[2]
                
                # Normalize angle to [-pi, pi]
                heading_err = (heading_err + math.pi) % (2 * math.pi) - math.pi
                
                # Proportional Control for Omega
                Kp = 4.0
                robot.omega = Kp * heading_err
                
                # Clamp omega
                robot.omega = max(min(robot.omega, 2.0), -2.0)
                
                # Constant velocity if heading is roughly correct
                if abs(heading_err) < math.pi / 2:
                    robot.velocity = 1.0 # 1 m/s
                else:
                    robot.velocity = 0.0 # Turn in place if wrong way
            
            # 2a. Reactive Collision Avoidance (Overrides velocity and replans)
            if check_collision(robot.pose, scan_points):
                robot.velocity = 0.0
                if replan_cooldown <= 0:
                    print("Collision warning! Replanning...")
                    path = planner.plan(robot.pose, final_goal)
                    if not path:
                        planning_status = -1.0
                    else:
                        planning_status = 1.0
                    replan_cooldown = 30 # Wait ~1 second at 30fps before replanning again
                else:
                    print("Collision warning! Obstacle ahead.")
            
        if replan_cooldown > 0:
            replan_cooldown -= 1

        # 2b. Send Telemetry Data
        # Calculate heading error for telemetry if path exists
        h_err = 0.0
        if path:
            target = path[0]
            target_heading = math.atan2(target[1] - robot.pose[1], target[0] - robot.pose[0])
            h_err = target_heading - robot.pose[2]
            h_err = (h_err + math.pi) % (2 * math.pi) - math.pi
        
        telemetry_queue.put((robot.velocity, robot.omega, h_err, planning_status))

        # 3. Physics Step
        robot.update(dt)
        
        # 4. Sensor Step
        scan_points = lidar.scan(robot.pose, truth_map)
        
        # 5. Mapping Step
        mapper.update(robot.pose, scan_points)
        
        # 6. Visualization Step
        screen.fill((20, 20, 20)) # Dark background
        
        # Draw Map
        # Iterate over the grid. 
        # Optimization: Only draw visible area? Or just draw all since grid is small (100x100).
        for r in range(GRID_CELLS):
            for c in range(GRID_CELLS):
                val = mapper.grid[r, c]
                if val != 0: 
                    color = (20, 20, 20)
                    if val > 0.5: color = (0, 0, 255) # Blue occupied
                    elif val < -0.5: color = (200, 200, 200) # Gray free
                    else: continue
                    
                    # Convert Grid -> World -> Screen
                    # World pos of cell center
                    wx = (c * CELL_SIZE) + (CELL_SIZE / 2)
                    wy = (r * CELL_SIZE) + (CELL_SIZE / 2)
                    
                    sx, sy = camera.world_to_screen(wx, wy)
                    # Width/Height of cell in screen px
                    # Use the same base_scale as Camera.world_to_screen (85% height fit)
                    base_scale = (camera.height * 0.85) / MAP_SIZE_METERS
                    size = int(CELL_SIZE * base_scale * camera.zoom) + 1 # +1 to avoid gaps
                    
                    pygame.draw.rect(screen, color, (sx - size//2, sy - size//2, size, size))

        # Draw Path
        if len(path) > 1:
            points = []
            for node in path:
                sx, sy = camera.world_to_screen(node[0], node[1])
                points.append((sx, sy))
            pygame.draw.lines(screen, (0, 255, 255), False, points, 3)

        # Draw Lidar Rays/Hits
        rx_s, ry_s = camera.world_to_screen(robot.pose[0], robot.pose[1])
        
        for p in scan_points:
            px_s, py_s = camera.world_to_screen(p[0], p[1])
            pygame.draw.line(screen, (50, 50, 0), (rx_s, ry_s), (px_s, py_s), 1)
            pygame.draw.circle(screen, (255, 0, 0), (px_s, py_s), 2)

        # Draw Robot (Two-Wheel Shape)
        # Base dimensions in meters
        robot_w = 0.5
        robot_h = 0.4
        wheel_w = 0.15
        wheel_h = 0.1
        
        # Coordinate system transformation
        def get_rotated_rect(center, w, h, angle):
            # center (wx, wy), w, h in meters
            # returns 4 points
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)
            
            p1 = (center[0] + (w/2)*cos_a - (h/2)*sin_a, center[1] + (w/2)*sin_a + (h/2)*cos_a)
            p2 = (center[0] - (w/2)*cos_a - (h/2)*sin_a, center[1] - (w/2)*sin_a + (h/2)*cos_a)
            p3 = (center[0] - (w/2)*cos_a + (h/2)*sin_a, center[1] - (w/2)*sin_a - (h/2)*cos_a)
            p4 = (center[0] + (w/2)*cos_a + (h/2)*sin_a, center[1] + (w/2)*sin_a - (h/2)*cos_a)
            return p1, p2, p3, p4

        # Draw Chassis
        chassis_pts = get_rotated_rect((robot.pose[0], robot.pose[1]), robot_w, robot_h, robot.pose[2])
        chassis_screen = [camera.world_to_screen(p[0], p[1]) for p in chassis_pts]
        pygame.draw.polygon(screen, (100, 100, 100), chassis_screen)
        pygame.draw.polygon(screen, (200, 200, 200), chassis_screen, 2) # outline

        # Draw Wheels
        # Left wheel
        lw_center = (robot.pose[0] - 0.05*math.cos(robot.pose[2]) + (robot_h/2)*math.sin(robot.pose[2]), 
                     robot.pose[1] - 0.05*math.sin(robot.pose[2]) - (robot_h/2)*math.cos(robot.pose[2]))
        # Right wheel
        rw_center = (robot.pose[0] - 0.05*math.cos(robot.pose[2]) - (robot_h/2)*math.sin(robot.pose[2]), 
                     robot.pose[1] - 0.05*math.sin(robot.pose[2]) + (robot_h/2)*math.cos(robot.pose[2]))
        
        for w_c in [lw_center, rw_center]:
            w_pts = get_rotated_rect(w_c, wheel_w, wheel_h, robot.pose[2])
            w_screen = [camera.world_to_screen(p[0], p[1]) for p in w_pts]
            pygame.draw.polygon(screen, (40, 40, 40), w_screen)

        # Draw heading indicator (Front of robot)
        hx = robot.pose[0] + (robot_w/2) * math.cos(robot.pose[2])
        hy = robot.pose[1] + (robot_w/2) * math.sin(robot.pose[2])
        hx_s, hy_s = camera.world_to_screen(hx, hy)
        pygame.draw.circle(screen, (0, 255, 0), (hx_s, hy_s), 4)

        pygame.display.flip()

    pygame.quit()
    telemetry_queue.put("STOP")
    telemetry_process.join(timeout=1)
    if telemetry_process.is_alive():
        telemetry_process.terminate()
    sys.exit()

if __name__ == "__main__":
    main()
