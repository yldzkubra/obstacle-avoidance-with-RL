#main.py
import pygame
from robot import Robot
from graphics import Graphics
from ultrasonic import Ultrasonic
from q_learning_agent import QLearningTable
import numpy as np


if __name__ == "__main__":
    # Map dimensions y x 
    MAP_DIMENSIONS = (775, 800)
    # Path to robot image
    ROBOT_PATH = "robot.png"
    robot_size = (80, 80)
    # Obstacles
    obstacles = [
        (50, 400, 20),
        (450, 300, 20),
        (400, 100, 20),
        (700, 100, 25),
        (500, 400, 32),
        (200, 500, 12),
        (50, 700, 25)
        
    ]

    # Initialize Graphics object
    gfx = Graphics(MAP_DIMENSIONS, ROBOT_PATH, obstacles)
    
    # Create Q-Learning agent
    x_bins = np.linspace(0, MAP_DIMENSIONS[1], num=10)
    y_bins = np.linspace(0, MAP_DIMENSIONS[0], num=10)
    RL = QLearningTable(x_bins, y_bins, actions=list(range(3)))  # Actions: forward, backward, left, right
    
    # Lidar sensor
    lidar_sensor_range = 80
    lidar_num_points = 180
    lidar = Ultrasonic(lidar_sensor_range, lidar_num_points, gfx.map)

    time_step = 0
    last_time = pygame.time.get_ticks()
   
    all_costs = []
    steps = []
    done = False
    for episode in range(10000):
        print("episode:",episode)
        # Robot parameters
        x0, y0, q0 = 100, 100, 0
        xg, yg = 650, 700
        robot = Robot( x0, y0, q0, xg, yg, 0.01 * 3779.52)
        observation = RL.discretize_state(100, 100)
        
        i=0
        cost = 0

        if done:
            print("finishh")
            break
     
        # Ana döngü içerisindeki eğitim döngüsü
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Calculate time step
            time_step = (pygame.time.get_ticks() - last_time) / 1000
            last_time = pygame.time.get_ticks()

            # Clear the map
            gfx.map.fill((0, 0, 0))

            # Draw obstacles and border
            gfx.draw_obstacles()
            gfx.draw_border()
            # Update robot kinematics
            robot.kinematics(time_step)
            # Draw robot
            gfx.draw_robot(robot.x0, robot.y0, robot.q0)
            # Sense obstacles
            point_cloud = lidar.sense_obstacles(robot.x0, robot.y0, robot.q0)
            # Avoid obstacles
            robot.avoid_obstacles(point_cloud)
            # Draw sensor data
            gfx.draw_sensor_data(point_cloud)

            
            # Choose action
            action = RL.choose_action(observation)
            # Calculate reward
            reward = RL.calculate_reward(robot.x0, robot.y0, robot.xg, robot.yg, robot.closest_obs)
            running, done = robot.step(action, time_step)
            
            # Get next observation
            next_observation =  RL.discretize_state(robot.x0, robot.y0)

            # Update Q-table
            cost += RL.learn(observation, action, reward, next_observation, robot.closest_obs)
            observation = next_observation

            i += 1
            if not running:
                steps += [i]
                all_costs += [i]
                break

            pygame.display.flip()
            pygame.display.update()



    