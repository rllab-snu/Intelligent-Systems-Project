#!/usr/bin/env python2
import os
import subprocess
import rospy
import rospkg

from std_msgs.msg import String
from sim2real.msg import Result, Query

import json
import yaml
import time
import curses
from datetime import date


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

        self.init_time = time.time()
        self.n_track = len(self.world_info)
        self.n_trial = 3
        self.n_query = 0
        self.n_finish = 0
        self.n_wait = 0
        self.n_running = 0
        self.query_queue = []
        self.query_database = []
        self.status = AVAILABLE
        self.time = time.time()
        self.sleep_time = 10.0

        self.team_sub = rospy.Subscriber("/team", String, self.callback_team)
        self.status_sub = rospy.Subscriber("/result", Result, self.callback_result)
        self.query_pub = rospy.Publisher("/query", Query, queue_size = 1)

        curses.wrapper(self.eval)
        # self.eval()


    def callback_team(self, data):
        team_name = data.data
        self.query_database.append({'id': self.n_query, 'team': team_name, 'count': 0, 'status': 'WAIT'})
        for trial in range(1, self.n_trial + 1):
            for i in range(self.n_track):
                world = self.world_info[i]
                q = {'id': self.n_query, 'trial': trial, 'team': team_name, 'world': world['name']}
                if trial == self.n_trial and i == self.n_track - 1:
                    q['exit'] = True
                else :
                    q['exit'] = False
                self.query_queue.append(q)
        self.n_query += 1
        self.n_wait += 1
        return

    def callback_result(self, data):
        # summary result
        # print("==============SUMMARY==============")
        # if data.success:
        #     print("[SUCCESS]")
        # else :
        #     print("[FAIL]")
        # print("[%04d, %d / %d] %s, %s" %(data.id, data.trial, self.n_trial, data.team, data.world))
        # print("===================================")
        self.query_database[data.id]['count'] += 1
        if self.query_database[data.id]['count'] == self.n_track * self.n_trial:
            self.query_database[data.id]['status'] = 'FINISH'
            self.n_finish += 1
            self.n_running -= 1
        self.time = time.time()
        self.status = SLEEP

    def get_status(self):
        if self.status == AVAILABLE:
            return "AVAILABLE"
        if self.status == SLEEP:
            return "SLEEP"
        if self.status == RUNNING:
            return "RUNNING"


    def eval(self, screen):
        curses.curs_set(0)    # Hide the cursor
        screen.nodelay(True)  # Don't block I/O calls
        while not rospy.is_shutdown():
            screen.refresh()
            time.sleep(0.1)
            screen.erase()
            s = "RLLAB@SNU   IS Project Evaluation Agent  STATUS: %s" %(self.get_status())
            screen.addstr(1, 0, s)
            runtime = int(time.time() - self.init_time)
            second = runtime % 60
            runtime = runtime / 60
            minute = runtime % 60
            runtime = runtime / 60
            hours = runtime % 24
            runtime = runtime / 24
            days = runtime
            s = "Date %s    RunTime: %02d day %02d hr %02d min %02d sec" %(date.today(), days, hours, minute, second)
            screen.addstr(2, 0, s)
            s = "[Query Info]  Total: %04d   Finish: %04d   Wait: %04d   Running: %04d" %(self.n_query, self.n_finish, self.n_wait, self.n_running)
            screen.addstr(3, 0, s)
            n_show = min(10, self.n_query)
            s = "ID         TEAM NAME      STATUS"
            screen.addstr(6, 0, s)
            for i in range(n_show):
                q = self.query_database[self.n_query - 1 - i]
                s = "%04d    %12s    %8s [%02d / %02d]" %(q['id'], q['team'], q['status'], q['count'], self.n_trial * self.n_track)
                screen.addstr(7 + i, 0, s)

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
                if self.query_database[rt.id]['status'] == 'WAIT':
                    subprocess.Popen("rosrun sim2real " + rt.name + "_project3.py > /dev/null 2>&1", shell = True)
                    time.sleep(5.0)
                    self.query_database[rt.id]['status'] = 'RUNNING'
                    self.n_wait -= 1
                    self.n_running += 1
                self.query_queue.pop(0)
                self.query_pub.publish(rt)

if __name__ == '__main__':
    with open(yaml_file) as file:
        args = yaml.load(file)
    EvalAgent(args)
