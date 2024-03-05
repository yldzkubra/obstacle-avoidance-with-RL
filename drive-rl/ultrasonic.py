#ultrasonic.py
import numpy as np
import pygame
import math


class Ultrasonic:
    def __init__(self, sensor_range, num_points, map):
        self.sensor_range = sensor_range
        self.num_points = num_points
        self.map_width, self.map_height = pygame.display.get_surface().get_size()
        self.map = map
        self.obstacle_color = (255, 255, 255)
    
    def sense_obstacles(self, x0, y0, q0):
        obstacles = []
        
        angles = np.linspace(0, 2*math.pi, self.num_points, endpoint=False)
        
        for angle in angles:
            x2 = x0 + self.sensor_range * math.cos(angle + q0)
            y2 = y0 + self.sensor_range * math.sin(angle + q0)
            
            for i in range(0, 100):
                u = i / 100
                x1 = int(x0 * (1 - u) + x2 * u)
                y1 = int(y0 * (1 - u) + y2 * u)

                if 0 < x1 < self.map_width and 0 < y1 < self.map_height:
                    
                    color = self.map.get_at((x1, y1))
                    self.map.set_at((x1, y1), (0, 196, 255))
                    
                    # obstacle color is black
                    if (color[0], color[1], color[2]) == self.obstacle_color:
                        obstacles.append([x1, y1])
                        break
        output_file = "reversed_path.txt"
        with open(output_file, 'w') as file:
            file.write(str(obstacles))
        
        return obstacles
                    
            