#graphics.py
import numpy as np
import math
import pygame

class Graphics:
    def __init__(self, dimensions, robot_img_path, obstacles):
        pygame.init()

        self.sensor_color = (231, 238, 0 )
        
        # map
        self.robot = pygame.image.load(robot_img_path)

        self.height, self.width = dimensions

        # window settings
        pygame.display.set_caption("Obstacle Avoidance")
        self.map = pygame.display.set_mode((self.width, self.height))

        # Create obstacles
        self.obstacles = obstacles
    
    def set_obstacles(self, obstacles):
        self.obstacles = obstacles

    def draw_robot(self, x0, y0, q0):
        rotated = pygame.transform.rotozoom(self.robot, math.degrees(q0), 1)
        rect = rotated.get_rect(center=(x0, y0))
        self.map.blit(rotated, rect)
     
        

    def draw_obstacles(self):
        for obstacle in self.obstacles:
            pygame.draw.circle(self.map, (255, 255, 255), obstacle[:2], obstacle[2])

    def draw_sensor_data(self, point_cloud):
        for point in point_cloud:
            pygame.draw.circle(self.map, self.sensor_color, point, 3, 0)
    
    def draw_border(self):
        pygame.draw.rect(self.map, (255, 255, 255), (0, 0, self.width, self.height), 10)
        pygame.draw.circle(self.map, (0,208, 255), (100,100), 5)
        pygame.draw.circle(self.map, (0, 208, 255), (650,700), 5)
    


    '''
     def draw_robot(self, x0, y0, q0, robot_size):
        # Calculate rotated square for the body of the robot
        rotated_rect = pygame.Surface(robot_size, pygame.SRCALPHA)
        rotated_rect.fill((0, 0, 0, 0))
        pygame.draw.rect(rotated_rect, (4, 52, 108), (0, 0, *robot_size))

        # Calculate positions for the wheels
        wheel_radius = min(robot_size) // 4
        wheel_positions = [
            (x0 - robot_size[0] // 2 + wheel_radius, y0 - robot_size[1] // 2 + wheel_radius),
            (x0 + robot_size[0] // 2 - wheel_radius, y0 - robot_size[1] // 2 + wheel_radius),
            (x0 - robot_size[0] // 2 + wheel_radius, y0 + robot_size[1] // 2 - wheel_radius),
            (x0 + robot_size[0] // 2 - wheel_radius, y0 + robot_size[1] // 2 - wheel_radius)
        ]

        # Draw wheels
        for pos in wheel_positions:
            pygame.draw.circle(rotated_rect, (146, 219, 0), pos, wheel_radius)

        # Rotate the square
        rotated = pygame.transform.rotate(rotated_rect, math.degrees(-q0))
        rect = rotated.get_rect(center=(x0, y0))
        self.map.blit(rotated, rect)

        
    '''

    