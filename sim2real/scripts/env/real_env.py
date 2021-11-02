#!/usr/bin/env python
from __future__ import print_function

from sensor_msgs.msg import LaserScan, JointState
from race.msg import drive_param
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Pose

import roslib
import rospy

from scipy.spatial.transform import Rotation
from gym.utils import seeding
from copy import deepcopy
from gym import spaces
import numpy as np
import random
import threading
import time
import gym
import sys

class Env(gym.Env):
  def __init__(self):
    rospy.init_node('get_state', anonymous=True)

    # maybe we need to add estop??
    self.drive_msg = drive_param()
    self.drive_msg.angle = 0.0
    self.drive_msg.velocity = 0.0
    self.angle_range = np.arange(40, 1080, 40) # -135 degree to 135 degree per 10 degrees    
    self.rate = rospy.Rate(10) #unit : hz
    self.limit_distance = 0.6
    self.max_step = 2000
    self.cur_step = 0
    self.t_last = time.time()
    self.track = '2'
    self.idx = 0
    self.estop = True

    # state & action
    self.sensor_value = np.zeros_like(self.angle_range, dtype=np.float32)
    self.rpm_data = 0.0
    self.steering = 0.0

    #state & action dimension
    self.num_action = 6 # left, straight, right + stop, go
    self.state_dim = len(self.sensor_value) + 2 # lidar + velocity + steering
    self.action_dim = 2
    #self.action_space = spaces.Discrete(self.num_action)
    self.action_space = spaces.Box(-np.ones(self.action_dim), np.ones(self.action_dim), dtype=np.float32)
    self.observation_space = spaces.Box(-np.inf*np.ones(self.state_dim), np.inf*np.ones(self.state_dim), dtype=np.float32)

    #publisher and subsrciber
    self.drive_pub = rospy.Publisher('/drive_parameters', drive_param , queue_size = 1)
    self.state_sub = rospy.Subscriber('/scan', LaserScan, self.state_callback) # -135 deg ~ 135 deg, length : 1081, 4 per 1 deg.
    self.rpm_sub = rospy.Subscriber('/ang_vel_data', Float32, self.rpm_callback)
    self.estop_sub = rospy.Subscriber('/estop', Bool, self.estop_callback)

    # define drive thread
    self.drive_thread = threading.Thread(target=self.drive_pub_thread, args=())
    self.drive_thread.daemon=True
    self.drive_thread_loop = True
    self.drive_thread_flag = True
    self.drive_thread.start()

    
  def state_callback(self, data):
    '''
    self.state = data
    self.state = {'ranges':data.ranges, 'intensities':data.intensities, 'angle_min':data.angle_min, 'angle_max':data.angle_max, \
                  'range_min':data.range_min, 'range_max':data.range_max}
    '''
    for i, idx in enumerate(self.angle_range):
      self.sensor_value[i] = np.clip(np.mean(data.ranges[idx-40:idx+40]), 0.0, 10.0)

  def rpm_callback(self, data):
    self.rpm_data = data.data

  def estop_callback(self, data):
    self.estop = data.data

  def drive_pub_thread(self):
    while self.drive_thread_loop:
      self.drive_pub.publish(self.drive_msg)
      self.drive_thread_flag = True
      time.sleep(0.01)
    self.drive_thread_flag = False

  def reset(self):
    self.cur_step = 0
    self.rpm_data = 0.0
    self.steering = 0.0
    self.drive_msg.angle = 0.0
    self.drive_msg.velocity = 0.0
    self.estop = True
    time.sleep(1)
    self.rate.sleep()

    print('\nreset environment')

    state = self.get_state()
    return state
    
  def drive(self, vel, steer):
    self.steering = steer
    self.drive_msg.angle = steer
    self.drive_msg.velocity = vel
    
  def sigmoid(self, x):
    return 2 * (1 / (1+np.exp(-x)))

  def step(self, action, vel):
    self.cur_step += 1

    steer_scale = 1.0
    vel_scale = 100.0

    velocity = vel
    steering = action
      
    steering = steering*steer_scale
    velocity = np.clip(velocity*vel_scale, 0.0, np.inf)
    
    self.drive(velocity, steering)
    self.rate.sleep()

    state = self.get_state()
    reward = vel
    done = False
    over = False
    reward += np.min(self.sensor_value) * 0.2
    if np.min(self.sensor_value) < self.limit_distance:
      done = True
      reward -= 10.0

    if self.cur_step >= self.max_step:
      print('done')
      done = True
      over = True

    if done:
      self.drive_msg.angle = 0.0
      self.drive_msg.velocity = 0.0

    return state, reward, done, over

  def get_state(self):
    state = np.concatenate([self.sensor_value/10.0, [self.rpm_data/100.0], [self.steering]])
    return state

  def seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def render(self, mode='human'):
    pass

  def close(self):
    self.drive_thread_loop = False
    while self.drive_thread_flag:
      time.sleep(0.001)
    self.drive_thread.join()