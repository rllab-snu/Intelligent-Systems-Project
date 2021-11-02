#!/usr/bin/env python2
import rospy
import rospkg

from std_msgs.msg import String
from sim2real.msg import Result, Query

import json
import yaml
import time


AVAILABLE = 0
SLEEP = 1
RUNNING = 2

project_path = rospkg.RosPack().get_path("sim2real")
yaml_file = project_path + "/config/eval.yaml"

class EvalAgent:
    def __init__(self, args):
        rospy.init_node('evaluation_agent', anonymous=True)
        # load world info
        self.world_info_path = project_path + "/worlds/" + args['world_name'] + ".json"
        with open(self.world_info_path, "r") as world_info:
            self.world_info = json.load(world_info)
        # world info is a list of world
        # each world is formatted {'name': {world_name}, 'offset': ['x', 'y']}

        self.n_track = len(self.world_info)
        self.n_trial = 1
        self.query_num = 0
        self.query_queue = []
        self.status = AVAILABLE
        self.time = time.time()
        self.sleep_time = 10.0

        self.team_sub = rospy.Subscriber("/team", String, self.callback_team)
        self.status_sub = rospy.Subscriber("/result", Result, self.callback_result)

        self.query_pub = rospy.Publisher("/query", Query, queue_size = 1)

        self.eval()


    def callback_team(self, data):
        team_name = data.data
        self.query_num += 1
        for trial in range(1, self.n_trial + 1):
            for i in range(self.n_track):
                world = self.world_info[i]
                q = {'id': self.query_num, 'trial': trial, 'team': team_name, 'world': world['name']}
                if trial == self.n_trial and i == self.n_track - 1:
                    q['exit'] = True
                else :
                    q['exit'] = False
                print("insert queue")
                print(q)
                self.query_queue.append(q)
        return

    def callback_result(self, data):
        """
        result contains
        int32 id
        int32 trial
        string team
        string world
        float32 elapsed_time
        int32 waypoints
        int32 n_waypoints
        bool success
        string fail_type
        """
        # summary result
        print("==============SUMMARY==============")
        if data.success:
            print("[SUCCESS]")
        else :
            print("[FAIL]")
        print("[%04d, %d / %d] %s, %s" %(data.id, data.trial, self.n_trial, data.team, data.world))
        print("===================================")
        self.time = time.time()
        self.status = SLEEP

    def eval(self):
        while not rospy.is_shutdown():
            time.sleep(0.1)
            if self.status == RUNNING:
                continue
            if self.status == SLEEP:
                if time.time() - self.time > self.sleep_time:
                    self.time = time.time()
                    self.status = AVAILABLE
                continue
            
            # publish
            if len(self.query_queue) > 0:
                self.status = RUNNING
                rt = Query()
                rt.id = self.query_queue[0]['id']
                rt.trial = self.query_queue[0]['trial']
                rt.name = self.query_queue[0]['team']
                rt.world = self.query_queue[0]['world']
                rt.exit = self.query_queue[0]['exit']
                self.query_queue.pop(0)
                self.query_pub.publish(rt)

if __name__ == '__main__':
    with open(yaml_file) as file:
        args = yaml.load(file)
    EvalAgent(args)
