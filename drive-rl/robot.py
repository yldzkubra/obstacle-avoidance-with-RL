#robot.py
import math
import math
import numpy as np

def distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1 - point2)

class Robot:
    def __init__(self, x0, y0, q0, xg, yg, width):

        self.meters_to_pixel = 3779.52
        self.w = width

        self.x0, self.y0, self.q0 = x0, y0, q0  # Initial position
        self.xg, self.yg = xg, yg  # Target pose

        self.vl = 0.04*self.meters_to_pixel
        self.vr = 0.04*self.meters_to_pixel

        
        self.max_speed = 0.05*self.meters_to_pixel
        self.min_speed = 0.05*self.meters_to_pixel
        
        self.closest_obs = None

    def avoid_obstacles(self, point_cloud):
        self.closest_obs = None
        dist = np.inf
        
        if len(point_cloud) > 1:
            for point in point_cloud:
                if dist > distance([self.x0, self.y0], point):
                    dist = distance([self.x0, self.y0], point)
                    self.closest_obs = (point, dist)


    def turn_left(self):
        self.q0 -= math.radians(10)  # 5 derece sola dön
        if self.q0 < -math.pi:
            self.q0 += 2 * math.pi

    def turn_right(self):
        self.q0 += math.radians(10)  # 5 derece sağa dön
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
    
    def step(self, action, time_step):
            running = True
            done = False
            #distance_to_goal = np.sqrt((self.xg - self.x0)**2 + (self.yg - self.y0)**2)
            #distance_to_obstacle = self.closest_obs[1] if self.closest_obs is not None else np.inf
            #print("action:",action)

            # Apply robot's movement based on chosen action
            if action == 0:
                    self.move_forward()
                    self.kinematics(time_step)
            elif action == 1:
                    self.turn_left()
                    self.kinematics(time_step)
            elif action == 2:
                    self.turn_right()
                    self.kinematics(time_step)

            if self.closest_obs is not None and self.closest_obs[1] < 45:
                running = False

            if self.check_distance() < 35:
                print("bitttiiii")
                done = True
                running = False
               

            return running, done

            
    
    def check_distance(self):
        distance_to_goal = np.sqrt((self.x0 - self.xg)**2 + (self.y0 - self.yg)**2)
        return distance_to_goal
           

    
    

       

