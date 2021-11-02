#!/usr/bin/env python2
from __future__ import print_function
import sys
import os
import rospkg
import rospy

######################################## PLEASE CHANGE TEAM NAME ########################################
TEAM_NAME = "RLLAB"
######################################## PLEASE CHANGE TEAM NAME ########################################
project_path = rospkg.RosPack().get_path("sim2real")
yaml_file = project_path + "/config/eval.yaml"
PATH = project_path + "/scripts"
sys.path.append(PATH)

from sim2real.msg import Result, Query

import gym
import env
import numpy as np
import math
import yaml
import time


def dist(waypoint, pos):
    return math.sqrt((waypoint[0] - pos.x) ** 2 + (waypoint[1] - pos.y) ** 2)

class PurePursuit:
    def __init__(self, args):
        rospy.init_node(TEAM_NAME + "_project1", anonymous=True, disable_signals=True)

        # env reset with world file
        self.env = gym.make('RCCar-v0')
        self.env.seed(1)
        self.env.unwrapped
        self.env.load(args['world_name'])
        self.track_list = self.env.track_list
        self.time_limit = 100.0

        ######################################## YOU CAN ONLY CHANGE THIS PART ########################################
        # TO DO
        """ 
        Setting hyperparameter
        Recommend tuning PID coefficient P->D->I order.
        Also, Recommend set Ki extremely low.
        """
        self.lookahead = 10
        self.prv_err = None
        self.cur_err = None
        self.sum_err = 0.0
        self.dt = 0.1
        self.min_vel = 1.5
        self.max_vel = 2.5
        self.Kp = 0.0
        self.Ki = 0.0
        self.Kd = 0.0
        ######################################## YOU CAN ONLY CHANGE THIS PART ########################################

        self.query_sub = rospy.Subscriber("/query", Query, self.callback_query)
        self.rt_pub = rospy.Publisher("/result", Result, queue_size = 1)

    def callback_query(self, data):
        rt = Result()
        START_TIME = time.time()
        is_exit = data.exit
        try:
            # if query is valid, start
            if data.name != TEAM_NAME:
                return
            
            if data.world not in self.track_list:
                END_TIME = time.time()
                rt.id = data.id
                rt.trial = data.trial
                rt.team = data.name
                rt.world = data.world
                rt.elapsed_time = END_TIME - START_TIME
                rt.waypoints = 0
                rt.n_waypoints = 20
                rt.success = False
                rt.fail_type = "Invalid Track"
                self.rt_pub.publish(rt)
                return

            print("[%s] START TO EVALUATE! MAP NAME: %s" %(data.name, data.world))
            self.env.reset(name = data.world)
            self.waypoints = self.env.waypoints_list[self.env.track_id]
            self.N = len(self.waypoints)
            self.prv_err = None
            self.cur_err = None
            self.sum_err = 0.0

            while True:
                if time.time() - START_TIME > self.time_limit:
                    END_TIME = time.time()
                    rt.id = data.id
                    rt.trial = data.trial
                    rt.team = data.name
                    rt.world = data.world
                    rt.elapsed_time = END_TIME - START_TIME
                    rt.waypoints = self.env.next_checkpoint
                    rt.n_waypoints = 20
                    rt.success = False
                    rt.fail_type = "Exceed Time Limit"
                    self.rt_pub.publish(rt)
                    print("EXCEED TIME LIMIT")
                    break
                

                cur_pos = self.env.get_pose()
                
                ######################################## YOU CAN ONLY CHANGE THIS PART ########################################
                # TO DO
                """ 
                1) Find nearest waypoint from a race-car
                2) Calculate error between the race-car heading and the direction vector between the lookahead waypoint and the race-car
                3) Determine input steering of the race-car using PID controller
                4) Calculate input velocity of the race-car appropriately in terms of input steering
                """
                
                input_steering = 0.0
                input_vel = 0.0
                _, _, done, logs = self.env.step([input_steering, input_vel])
                ######################################## YOU CAN ONLY CHANGE THIS PART ########################################


                if done:
                    END_TIME = time.time()
                    rt.id = data.id
                    rt.trial = data.trial
                    rt.team = data.name
                    rt.world = data.world
                    rt.elapsed_time = END_TIME - START_TIME
                    rt.waypoints = logs['checkpoints']
                    rt.n_waypoints = 20
                    rt.success = True if logs['info'] == 3 else False
                    rt.fail_type = ""
                    if logs['info'] == 1:
                        rt.fail_type = "Collision"
                    if logs['info'] == 2:
                        rt.fail_type = "Exceed Time Limit"
                    self.rt_pub.publish(rt)
                    break
        except:
            END_TIME = time.time()
            rt.id = data.id
            rt.trial = data.trial
            rt.team = data.name
            rt.world = data.world
            rt.elapsed_time = END_TIME - START_TIME
            rt.waypoints = 0
            rt.n_waypoints = 20
            rt.success = False
            rt.fail_type = "Script Error"
            self.rt_pub.publish(rt)

        if is_exit:
            rospy.signal_shutdown("End query")
        
        return

if __name__ == '__main__':
    with open(yaml_file) as file:
        args = yaml.load(file)
    PurePursuit(args)
    rospy.spin()

