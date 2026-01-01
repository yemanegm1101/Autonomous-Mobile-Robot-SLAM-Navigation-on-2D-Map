import math
import numpy as np
from config import *

class Robot:
    def __init__(self, x, y, theta):
        self.pose = np.array([x, y, theta], dtype=float)
        self.velocity = 0.0
        self.omega = 0.0 # Angular velocity

    def update(self, dt):
        """
        Differential Drive Kinematics
        Replicates: newX = prevPose.x + Math.cos(newTheta) * speed * dt
        """
        self.pose[2] += self.omega * dt
        self.pose[2] = (self.pose[2] + math.pi) % (2 * math.pi) - math.pi # Normalize

        self.pose[0] += self.velocity * math.cos(self.pose[2]) * dt
        self.pose[1] += self.velocity * math.sin(self.pose[2]) * dt

        # Boundary checks
        self.pose[0] = np.clip(self.pose[0], 0, MAP_SIZE_METERS)
        self.pose[1] = np.clip(self.pose[1], 0, MAP_SIZE_METERS)
