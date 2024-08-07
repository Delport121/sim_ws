from abc import abstractmethod
from argparse import Namespace
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import PoseWithCovarianceStamped
from real_car.DriveRosNode import Drive
import time
import math
import torch     
import yaml
import datetime


import torch
import torch.nn as nn
import torch.nn.functional as F

import numpy as np
from copy import copy

NUMBER_SCANS = 1
NUMBER_BEAMS = 28
MAX_SPEED = 3.5
MAX_STEER = 0.4
RANGE_FINDER_SCALE =10 
NN_LAYER_1 = 150
NN_LAYER_2 = 150

class DoublePolicyNet(nn.Module):
    def __init__(self, state_dim, act_dim):
        super(DoublePolicyNet, self).__init__()  
        self.fc1 = nn.Linear(state_dim, NN_LAYER_1)
        self.fc2 = nn.Linear(NN_LAYER_1, NN_LAYER_2)
        self.fc_mu = nn.Linear(NN_LAYER_2, act_dim)
    
    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        mu = torch.tanh(self.fc_mu(x))
        return mu

def extract_scan(obs):
    scan = np.array(obs['scan']) 
    inds = np.linspace(0, len(scan)-1, 40).astype(int) #change 40 to number beams after this test 
    scan = scan[inds]
    scan = scan[6:34]
    #scan = scan[14:66] # remove behind fov
    scan = np.clip(scan/RANGE_FINDER_SCALE, 0, 1)
    
    #scan = scan[::-1]
    #right = np.average(scan[3:5])
    #left = np.average(scan[15:17])
    right = np.average(scan[0:3])
    left = np.average(scan[25:28])
    #right = np.average(scan[0:6])
    #left = np.average(scan[46:53])    
    diff= (right - left) * 1
    # print(scan)

    return scan,diff

class TestTD3:
    def __init__(self,filename,directory):
        self.action_space = 2
        self.state_space = NUMBER_BEAMS + 3 
        self.agent = DoublePolicyNet(self.state_space, self.action_space)
        model_path = f'{directory}{filename}_actor.pth'
        self.agent.load_state_dict(torch.load(model_path))

    def act(self, state):
        state = torch.FloatTensor(state.reshape(1,-1))
        action = self.agent(state).data.numpy().flatten()

        
        return action
      

class Agent(Drive):
    def __init__(self):
        super().__init__("drive_agent")
        agent_name = 'place holder'
        filename = 'TD3_V27_new_track_0dict'
        print('no load')
        self.n_scans = 1
        self.scan_buffer = np.zeros((self.n_scans, NUMBER_BEAMS))
        # self.agent = DoublePolicyNet(self.state_space, self.action_space)
        path = '/home/ruan/f1tenth_pf_ws/src/real_car/real_car'
        
        self.agent = TestTD3(filename,path)
        # self.state_space = NUMBER_BEAMS 
        print('load')
        print(filename)


    def action_process(self, observation):
        nn_obs = self.process_observation(observation)
        nn_action = self.agent.act(nn_obs)
        #print(nn_obs)
        #print(nn_action)
        self.steering_angle = nn_action[0]*MAX_STEER
        self.speednoscale = nn_action[1] 
        self.speed = (nn_action[1] + 1) * (MAX_SPEED  / 2 - 0.5) + 1
        self.speed = min(self.speed, MAX_SPEED) # cap the speed
        #self.speed = (nn_action[1])
        #if self.speed < 0:
        #	self.speed = 0.1
        #elif self.speed > 0:
        #	self.speed = self.speed*MAX_SPEED
        #if observation['state'] < 1:
        #	action = np.array([0, 2])
        #else:	
        action = np.array([self.steering_angle, self.speed])
        
        return action
    
    def process_observation(self, obs):
        scan,diff = extract_scan(obs)
        if self.scan_buffer.all() ==0: # first reading
            for i in range(self.n_scans):
                self.scan_buffer[i, :] = scan 
        else:
            self.scan_buffer = np.roll(self.scan_buffer, 1, axis=0)
            self.scan_buffer[0, :] = scan
        final_scan = np.reshape(self.scan_buffer, (NUMBER_BEAMS * self.n_scans))
        nn_obs = final_scan
        #speed = obs['state'][3] 
        #speed = obs['state']/ MAX_SPEED
        #steer = obs['state'][1]
        speed = self.speed/ MAX_SPEED
        steer = self.steering_angle/MAX_STEER
        nn_obs = np.append(nn_obs, speed)
        nn_obs = np.append(nn_obs,steer)
        nn_obs = np.append(nn_obs,diff)
        #print(nn_obs)

        return nn_obs
    

def main(args=None):
    rclpy.init(args=args)
    node = Agent()
    # node.run_lap()
    rclpy.spin(node)

if __name__ == '__main__':
      main()
