from abc import abstractmethod
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import math
from scipy.ndimage import uniform_filter1d
from geometry_msgs.msg import TransformStamped
import tf2_ros

import numpy as np
from copy import copy
from argparse import Namespace

import os
import datetime
import csv
import yaml
STOP_DISTANCE = 0.25

class Drive(Node):
    def __init__(self,node_name):
        super().__init__(node_name)

        self.speed = 0
        self.steering_angle = 0.0
        self.speednoscale = 0
        self.scan = None 
        self.declare_parameter('max_accepted_distance', 10.0)
        self.declare_parameter('truncated_coverage_angle', 180.0)
        self.declare_parameter('smoothing_filter_size', 3)
        self.truncated = False
        self.action_buffer = np.zeros(())
        self.done = False

        self.lidar_subscriber = self.create_subscription(LaserScan,'/scan', self.scan_callback,10)
        self.action_publisher = self.create_publisher(AckermannDriveStamped,'drive',10)
        self.odom_sub = self.create_subscription(Odometry, "/pf/pose/odom", self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)

    def odom_callback(self, odom_msg):
        # Extract the linear velocity of the car from the odometry message and store it
        self.speed = odom_msg.twist.twist.linear.x   

    def scan_callback(self, scan_msg):
        # Callback function for processing lidar scans
        # First time this is run, we calculate the truncated indices based on the desired angle coverage
        # if not self.truncated:
        #     truncated_indices = self.truncated_start_and_end_indices(scan_msg, self.get_parameter(
        #         'truncated_coverage_angle').get_parameter_value().double_value)
        #     self.get_logger().info(f"Truncated Indices: {truncated_indices}")
        #     self.truncated_start_index, self.truncated_end_index = truncated_indices
        #     self.truncated = True

        filtered_ranges = self.preprocess_lidar_scan(scan_msg)
        # filtered_ranges = np.array(scan_msg.ranges)
        # filtered_ranges = filtered_ranges[180:900]
        self.scan = filtered_ranges
        # print('SCAN RECIEVED')
        closest_index = self.minimum_element_index(filtered_ranges)
        closest_range = filtered_ranges[closest_index]
        if closest_range < STOP_DISTANCE:
            self.done = True
        else:
            self.done = False

        self.drive_callback()

    def apply_smoothing_filter(self, input_vector, smoothing_filter_size):
        # Apply a uniform 1D filter to smooth the input vector, which helps to mitigate noise in the lidar data
        return uniform_filter1d(input_vector, size=smoothing_filter_size, mode='nearest')   

    def preprocess_lidar_scan(self, scan_msg):
        # Preprocess the lidar scan data
        # Convert NaNs to 0 for processing and cap the max range to a set value
        ranges = np.array(scan_msg.ranges)
        ranges[np.isnan(ranges)] = 0.0
        # ranges[ranges > self.get_parameter('max_accepted_distance').get_parameter_value().double_value] = self.get_parameter('max_accepted_distance').get_parameter_value().double_value
        # # Apply the smoothing filter
        return self.apply_smoothing_filter(ranges,self.get_parameter('smoothing_filter_size').get_parameter_value().integer_value)
    
    # def truncated_start_and_end_indices(self, scan_msg, truncation_angle_coverage):
    #     # Calculate start and end indices for the truncated view based on the desired coverage angle
    #     truncated_range_size = int(
    #         truncation_angle_coverage / (scan_msg.angle_max - scan_msg.angle_min) * len(scan_msg.ranges))
    #     start_index = len(scan_msg.ranges) // 2 - truncated_range_size // 2
    #     end_index = len(scan_msg.ranges) // 2 + truncated_range_size // 2
    #     return start_index, end_index 
    
    def minimum_element_index(self, input_vector):
        # Find the index of the minimum element in the input vector
        return int(np.argmin(input_vector))
    
    def create_observation(self):

        observation = {}
        observation["scan"] = self.scan
        if observation["scan"] is None: observation["scan"] = np.zeros(1080)

        state = np.array([self.speed])
        # self.broadcast_transform(state[:3])
        observation['state'] = state
        # print(state)

        return observation
    
    def send_drive_message(self, action):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = float(action[1])
        # print('action')
        # print(action)
        drive_msg.drive.steering_angle = float(action[0])
        self.drive_pub.publish(drive_msg)

    def drive_callback(self):
        observation = self.create_observation()
        action = self.action_process(observation)
        if self.done:
            action = np.array([0,0])

        self.send_drive_message(action)

    @abstractmethod
    def action_process(self, observation):
        """
            Use the observation to calculate an action that is returned
        """
        raise NotImplementedError
    
