#q_learning_agnet.py
import numpy as np
import pandas as pd

class QLearningTable:
    def __init__(self, x_bins, y_bins, actions, learning_rate=0.01, reward_decay=0.9, e_greedy=0.9):
        self.x_bins = x_bins
        self.y_bins = y_bins
        self.actions = actions
        self.lr = learning_rate
        self.gamma = reward_decay
        self.epsilon = e_greedy
        
        # Creating full Q-table for all cells
        self.q_table = pd.DataFrame(columns=self.actions, dtype=np.float64)
        # Creating Q-table for cells of the final route
        self.q_table_final = pd.DataFrame(columns=self.actions, dtype=np.float64)

       
    def discretize_state(self, x, y):
        x_bin = np.digitize(x, self.x_bins)
        y_bin = np.digitize(y, self.y_bins)
        return f"{x_bin}_{y_bin}"     
    
    
    def choose_action(self, observation):
        self.check_state_exist(observation)
        if np.random.uniform() < self.epsilon:
            state_action = self.q_table.loc[observation, :]
            state_action = state_action.reindex(np.random.permutation(state_action.index))
            action = state_action.idxmax()
        else:
            # Choosing random action - left 10 % for choosing randomly
            action = np.random.choice(self.actions)
        return action
    
        
    def calculate_reward(self, x0, y0, xg, yg, closest_obs):
        # Hedefe olan mesafeyi hesapla
        distance_to_goal = np.sqrt((x0 - xg)**2 + (y0 - yg)**2)
        
        # Hedefe ulaşma ödülü
        goal_reward = 1.0 / (1.0 + distance_to_goal)

        # Engel cezası
        obstacle_penalty = 0
        if closest_obs is not None:
            distance_to_obstacle = closest_obs[1]
            if distance_to_obstacle < 45:  # Engelden 20 birim veya daha yakında
                obstacle_penalty = -0.1   # Engelden kaçınmada başarısızlık cezası

        # Toplam ödül
        reward = goal_reward + obstacle_penalty 
        return reward


    def learn(self, state, action, reward, next_state, closest_obs):
        self.check_state_exist(next_state)
        q_predict = self.q_table.loc[state, action]

        if closest_obs is not None and closest_obs[1] < 45:  # obstacle encountered
            q_target = reward  # Assign reward directly
        else:
            q_target = reward + self.gamma * self.q_table.loc[next_state, :].max()
        
        # Update Q-value using Q-learning update rule
        self.q_table.loc[state, action] += self.lr * (q_target - q_predict)
        return self.q_table.loc[state, action]

    
    def check_state_exist(self, state):
        if state not in self.q_table.index:
            self.q_table = self.q_table.append(
                pd.Series([0] * len(self.actions), index=self.q_table.columns, name=state)
            )
    
    





'''
    def choose_action(self, observation):
        self.check_state_exist(observation)
        if np.random.uniform() < self.epsilon:
            state_action = self.q_table.loc[observation, :]
            state_action = state_action.reindex(np.random.permutation(state_action.index))
            action = state_action.idxmax()
        else:
            state_action = self.q_table.loc[observation, :]
            state_action = state_action.reindex(np.random.permutation(state_action.index))
            action = state_action.idxmax()
            # Choosing random action - left 10 % for choosing randomly
            action = np.random.choice(self.actions)
        return action
'''      
   
