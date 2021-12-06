#!/usr/bin/env python
from __future__ import print_function

from gazebo_msgs.msg import ModelState, ModelStates
from sensor_msgs.msg import LaserScan, JointState
from race.msg import drive_param
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose

import roslib
import rospy
import rospkg

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
import json
import math

project_path = rospkg.RosPack().get_path("sim2real")

class Env(gym.Env):
  def __init__(self, track_id = None):
    # rospy.init_node('get_state', anonymous=True)
    self.drive_msg = drive_param()
    self.drive_msg.angle = 0.0
    self.drive_msg.velocity = 0.0
    self.angle_range = np.arange(40, 1080, 40) # -135 degree to 135 degree per 10 degrees    
    self.rate = rospy.Rate(10) #unit : hz
    self.arrive_distance = 2.0
    self.limit_distance = 0.4
    self.max_step = 10000
    self.cur_step = 0
    self.idx = 0
    # state & action
    self.sensor_value = np.zeros(1081, dtype = np.float32)
    self.rpm_data = 0.0
    self.steering = 0.0
    self.steer_scale = 1.0
    self.vel_scale = 100.0

    #state & action dimension
    self.num_action = 6 # left, straight, right + stop, go
    self.state_dim = len(self.sensor_value) + 2 # lidar + velocity + steering
    self.action_dim = 2
    #self.action_space = spaces.Discrete(self.num_action)
    self.action_space = spaces.Box(-np.ones(self.action_dim), np.ones(self.action_dim), dtype=np.float32)
    self.observation_space = spaces.Box(-np.inf*np.ones(self.state_dim), np.inf*np.ones(self.state_dim), dtype=np.float32)
    #publisher and subsrciber
    self.drive_pub = rospy.Publisher('/drive_parameters', drive_param , queue_size = 1)
    self.init_pub = rospy.Publisher('/gazebo/set_model_state', ModelState , queue_size = 1)
    self.state_sub = rospy.Subscriber('/scan', LaserScan, self.state_callback) # -135 deg ~ 135 deg, length : 1081, 4 per 1 deg.
    self.rpm_sub = rospy.Subscriber('/ang_vel_data', Float32, self.rpm_callback)
    self.pos_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.pos_callback)
    # define drive thread
    self.drive_thread = threading.Thread(target=self.drive_pub_thread, args=())
    self.drive_thread.daemon=True
    self.drive_thread_loop = True
    self.drive_thread_flag = True
    self.drive_thread.start()

    self.init_quat = Rotation.from_rotvec([random.uniform(-0.1,0.1), random.uniform(-0.1,0.1), np.pi/2]).as_quat()
    
    self.cur_pos = Pose()

    model_states = rospy.wait_for_message('gazebo/model_states', ModelStates)
    self.init_state = ModelState()
    idx = model_states.name.index('racecar')
    self.init_state.model_name = model_states.name[idx]
    self.init_state.reference_frame = ''

  def load(self, world_name):
    self.world_info_path = project_path + "/worlds/" + world_name + ".json"
    with open(self.world_info_path, "r") as world_info:
      self.world_info = json.load(world_info)
    self.track_list = []
    self.offset_list = []
    self.waypoints_list = []
    self.n_tracks = len(self.world_info)
    for track in self.world_info:
      self.track_list.append(track['name'])
      self.offset_list.append(track['offset'])
    
    for t in range(self.n_tracks):
      name = self.track_list[t]
      offset = self.offset_list[t]
      file_path = project_path + "/worlds/waypoints_augmented/" + name + "_aug_waypoints.txt"
      lines = []
      with open(file_path, "r") as f:
          for line in f:
              lines.append(line.split())
      N = len(lines)
      waypoints = []
      for i in range(N):
          waypoints.append(np.array([offset[0] + float(lines[i][0]), offset[1] + float(lines[i][1])]))
      self.waypoints_list.append(waypoints)
    
    self.n_checkpoints = 20
    self.checkpoints_list = []
    for t in range(self.n_tracks):
      checkpoints = []
      N = len(self.waypoints_list[t])
      for i in range(self.n_checkpoints):
        checkpoints.append(self.waypoints_list[t][int(N*(i+1)/self.n_checkpoints)-1])
      self.checkpoints_list.append(checkpoints)
        

  def state_callback(self, data):
    '''
    self.state = data
    self.state = {'ranges':data.ranges, 'intensities':data.intensities, 'angle_min':data.angle_min, 'angle_max':data.angle_max, \
                  'range_min':data.range_min, 'range_max':data.range_max}
    '''
    for i in range(1081):
      self.sensor_value[i] = data.ranges[i]

  def rpm_callback(self, data):
    self.rpm_data = data.data

  def pos_callback(self, data):
    self.cur_pos = data.pose[data.name.index('racecar')]

  def drive_pub_thread(self):
    while self.drive_thread_loop:
      self.drive_pub.publish(self.drive_msg)
      self.drive_thread_flag = True
      time.sleep(0.01)
    self.drive_thread_flag = False

  def reset(self, name = None):
    self.cur_step = 0
    self.rpm_data = 0.0
    self.steering = 0.0
    self.drive_msg.angle = 0.0
    self.drive_msg.velocity = 0.0
    self.next_checkpoint = 0
    time.sleep(0.1)
    self.rate.sleep()
    valid = False
    self.track_id = -1
    for i in range(self.n_tracks):
      if self.track_list[i] == name:
        valid = True
        self.track_id = i
    if not valid:
      print("invalid track name")

    self.init_state.pose.position.x = self.offset_list[self.track_id][0]
    self.init_state.pose.position.y = self.offset_list[self.track_id][1]
    self.init_state.pose.position.z = 0.01
    self.init_state.pose.orientation.x = self.init_quat[0]
    self.init_state.pose.orientation.y = self.init_quat[1]
    self.init_state.pose.orientation.z = self.init_quat[2]
    self.init_state.pose.orientation.w = self.init_quat[3]
    self.init_pub.publish(self.init_state)
    time.sleep(1)
    self.rate.sleep()
    state = self.get_state()

    return state
    
  def drive(self, vel, steer):
    self.steering = steer
    self.drive_msg.angle = steer
    self.drive_msg.velocity = vel

  def is_collision(self):
    for _, idx in enumerate(self.angle_range):
      if np.clip(np.mean(self.sensor_value[idx-40:idx+40]), 0.0, 10.0) < self.limit_distance:
        return True
    return False

  def is_arrive_checkpoint(self):
    if self.next_checkpoint == self.n_checkpoints:
      return False
    p = self.cur_pos.position
    c = self.checkpoints_list[self.track_id][self.next_checkpoint]
    if math.sqrt((p.x - c[0]) ** 2 + (p.y - c[1]) ** 2) < self.arrive_distance:
      return True
    return False

  def step(self, action):
    steering = action[0]
    velocity = action[1]
    self.cur_step += 1
    steering = steering * self.steer_scale
    velocity = np.clip(velocity * self.vel_scale, 0.0, np.inf)
    self.drive(velocity, steering)
    self.rate.sleep()
    state = self.get_state()
    reward = velocity / self.vel_scale
    done = False
    logs = {'info': 0}
    reward += np.min(self.sensor_value) * 0.2

    if self.is_collision():
      done = True
      logs['info'] = 1
      reward -= 10.0

    if self.cur_step >= self.max_step:
      done = True
      logs['info'] = 2

    if self.is_arrive_checkpoint():
      self.next_checkpoint += 1

    logs['checkpoints'] = self.next_checkpoint
    if self.next_checkpoint == self.n_checkpoints:
      done = True
      logs['info'] = 3

    if done:
      self.drive_msg.angle = 0.0
      self.drive_msg.velocity = 0.0
    return state, reward, done, logs

  def get_state(self):
    state = np.concatenate([self.sensor_value, [self.rpm_data/100.0], [self.steering]])
    return state

  def get_pose(self):
    return self.cur_pos

  def get_track_list(self):
    return self.track_list

  def seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def render(self, mode='human'):
    pass

  def close(self):
    self.drive_thread_loop = False
    while self.drive_thread_flag:
      time.sleep(0.01)
    self.drive_thread.join()
