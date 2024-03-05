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

        self.x0 += ((self.vl + self.vr)/2) * math.cos(self.q0) *time_step
        self.y0 -= ((self.vl + self.vr)/2) * math.sin(self.q0) *time_step
        
        self.q0 += (self.vr - self.vl) / self.w * time_step
        
        if self.q0 > 2*math.pi or self.q0 < -2*math.pi:
            self.q0 = 0
        
        self.vr = max(min(self.max_speed, self.vr), self.min_speed)
        self.vl = max(min(self.max_speed, self.vl), self.min_speed)



def choose_action(self, x0, y0, xg, yg, closest_obs):
        # Epsilon değeri
        #self.epsilon *= self.epsilon_decay  # Epsilondaki azalmayı uygula
        epsilon = self.epsilon

        # Hedefe olan mesafeyi hesapla
        distance_to_goal = np.sqrt((xg - x0)**2 + (yg - y0)**2)
        
        distance_to_obstacle = closest_obs[1] if closest_obs is not None else np.inf
        
        # Eğer epsilon'dan küçük bir rastgele sayı üretilirse keşfetme gerçekleşecek
        if np.random.uniform() < epsilon:

            if distance_to_goal > 25 and distance_to_obstacle > 45:
                action = 0  # İleri gitme eylemini seç
            elif distance_to_goal <= 25 and distance_to_obstacle > 45:
                action = 0  # İleri gitme eylemini seç
            elif distance_to_obstacle < 45:
                if np.random.uniform() < 0.5:  # Rastgele 0 ile 1 arasında bir sayı üretip, 0.5'ten küçükse
                    action = 2  # Sağa dönme eylemini seç
                else:
                    action = 3  # Sola dönme eylemini seç
            else:
                action = 1  # Geri gitme eylemini seç
            print(distance_to_obstacle)
            print(action)
            return action
        '''

        else:
            # Eğer epsilon'dan büyük bir rastgele sayı üretilirse çıkarım gerçekleşecek
            # Q tablosundaki en yüksek değere sahip eylemi seç
            state = self.discretize_state(x0, y0)
            self.check_state_exist(state)
            state_actions = self.q_table.loc[state, :]
            action = np.random.choice(state_actions[state_actions == np.max(state_actions)].index)
            print("---------------------------")
        '''
    
    
       
    

       

