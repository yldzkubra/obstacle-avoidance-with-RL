#robot.py
import math
import pygame
from math import sqrt, atan2, sin, cos
import numpy as np

def distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1 - point2)

class Robot:
    def __init__(self, R, L, x0, y0, q0, xg, yg, Kv, Kh, Ki,width):

        self.meters_to_pixel = 3779.52
        self.w = width

        self.R = R  # Wheel radius
        self.L = L  # Base length
        self.x0, self.y0, self.q0 = x0, y0, q0  # Initial position
        self.xg, self.yg = xg, yg  # Target pose
        self.Kv = Kv  # Velocity gain
        self.Kh = Kh  # Heading gain
        self.Ki = Ki
        self.E = 0   # Cumulative error

        self.vl = 0.01*self.meters_to_pixel
        self.vr = 0.01*self.meters_to_pixel

        
        self.max_speed = 0.02*self.meters_to_pixel
        self.min_speed = 0.01*self.meters_to_pixel#
        
        self.min_obs_dist = 56
        self.count_down = 5

        self.path = []
        self.closest_obs = None

    def avoid_obstacles(self, point_cloud, time_step):
        dist = np.inf
        self.closest_obs = None
        if len(point_cloud) > 1:
            for point in point_cloud:
                if dist > distance([self.x0, self.y0], point):
                    dist = distance([self.x0, self.y0], point)
                    self.closest_obs = (point, dist)  
            '''
            if self.closest_obs[1] < self.min_obs_dist and self.count_down > 0:
                self.count_down -= time_step
                self.move_backward()
            else:
                self.count_down = 5
                self.move_forward()  
           '''
    def avoid_obstacles(self, point_cloud, time_step):
        dist = np.inf
        if len(point_cloud) > 1:
            for point in point_cloud:
                if dist > distance([self.x0, self.y0], point):
                    dist = distance([self.x0, self.y0], point)
                    self.closest_obs = (point, dist)
            if self.closest_obs[1] < self.min_obs_dist and self.count_down > 0:
                self.count_down -= time_step
                self.move_backward()
            else:
                self.count_down = 5
                self.move_forward()
       

    
    def turn_left(self):
        self.q0 -= math.radians(5)  # 5 derece sola dön
        if self.q0 < -math.pi:
            self.q0 += 2 * math.pi

    def turn_right(self):
        self.q0 += math.radians(5)  # 5 derece sağa dön
        if self.q0 > math.pi:
            self.q0 -= 2 * math.pi
            
    def move_backward(self):
        self.vr = -self.min_speed
        self.vl = -self.min_speed/2
        
    def move_forward(self):
        self.vr = self.min_speed
        self.vl = self.min_speed

    def kinematics(self, time_step):
        
        ev = sqrt((self.xg - self.x0) ** 2 + (self.yg - self.y0) ** 2)
        v = self.Kv * ev

        d_x = self.xg - self.x0
        d_y = self.yg - self.y0
        g_theta = atan2(d_y, d_x)
        e = atan2(sin(g_theta - self.q0), cos(g_theta - self.q0))
        e_P = e
        e_I = self.E + e
        w = self.Kh * e_P + self.Ki * e_I

        self.E = self.E + e

        self.x0 += ((self.vr + self.vl)/2) * cos(self.q0) * time_step
        self.y0 -= ((self.vr + self.vl)/2) * sin(self.q0) * time_step
        self.q0 += (self.vr - self.vl)  / self.L * time_step
       

        self.path.append((self.x0, self.y0))

        self.vr =  v + (w * self.L) / 2 
        self.vl =  v - (w * self.L) / 2 

        if ev < 3: 
            print(ev) 
            self.vr = 0
            self.vl = 0
            print("the end")

        if self.q0 > 2*math.pi or self.q0 < -2*math.pi:
            self.q0 = 0
        
    
        