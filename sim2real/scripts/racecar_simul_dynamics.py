#!/usr/bin/env python

from sensor_msgs.msg import LaserScan, JointState
from race.msg import drive_param
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Bool
import numpy as np
import threading
import rospy

class RaceCar:
    def __init__(self):
        rospy.init_node('dynamics', anonymous=True)

        # subscriber
        rospy.Subscriber("/drive_parameters", drive_param, self.drive_callback)
        rospy.Subscriber('/racecar/joint_states', JointState, self.rpm_callback)
        rospy.Subscriber('/eStop', Bool, self.eStop_callback)
        rospy.Subscriber('/racecar/reset', Bool, self.reset_callback)

        self.reset()


    def reset(self):
        # set kinematics & dynamics value
        self.body_length = rospy.get_param('/racecar/kinematics/body_length')
        self.body_width = rospy.get_param('/racecar/kinematics/body_length')
        self.wheel_radius = rospy.get_param('/racecar/kinematics/wheel_radius')
        self.gear_ratio = rospy.get_param('/racecar/kinematics/gear_ratio')
        self.tau_motor = rospy.get_param('/racecar/dynamics/tau_motor')
        self.tau_steer = rospy.get_param('/racecar/dynamics/tau_steer')
        self.scale_steer = rospy.get_param('/racecar/dynamics/scale_steer')

        # publisher
        self.pub_vel_left_rear_wheel = rospy.Publisher('/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_right_rear_wheel = rospy.Publisher('/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_left_front_wheel = rospy.Publisher('/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_right_front_wheel = rospy.Publisher('/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_pos_left_steering_hinge = rospy.Publisher('/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
        self.pub_pos_right_steering_hinge = rospy.Publisher('/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)
        self.pub_ang_vel_data = rospy.Publisher('/ang_vel_data', Float32, queue_size=1)
        self.pub_ang_data = rospy.Publisher('/ang_data', Float32, queue_size=1)

        # set taget value & current value & error value
        self.target_steer_ang = 0.0 # radian
        self.target_motor_vel = 0.0 # radian / second
        self.cur_steer_ang = 0.0
        self.cur_motor_vel = 0.0
        self.cur_motor_ang = 0.0
        self.err_motor_vel = self.target_motor_vel - self.cur_motor_vel
        self.err_steer_ang = self.target_steer_ang - self.cur_steer_ang

        # timer thread
        self.is_timer_run = True
        self.timer_period = 0.01
        self.timer_rate = rospy.Rate(1.0/self.timer_period)
        self.timer_thread_id = threading.Thread(target=self.timer_thread)
        self.timer_thread_id.daemon=True
        self.timer_thread_id.start()

    def reset_callback(self, data):
        if data.data:
            self.is_timer_run = False
            self.timer_thread_id.join()
            self.reset()
    
    def rpm_callback(self, data):
        idx1 = data.name.index('left_rear_wheel_joint')
        idx2 = data.name.index('right_rear_wheel_joint')

        #wheel_vel = (data.velocity[idx1] + data.velocity[idx2])/2
        #self.cur_motor_vel = self.gear_ratio * wheel_vel

        wheel_ang = (data.position[idx1] + data.position[idx2])/2
        self.cur_motor_ang = self.gear_ratio * wheel_ang

    def drive_callback(self, data):
        self.target_steer_ang = data.angle * self.scale_steer
        self.target_motor_vel = data.velocity
        self.err_motor_vel = self.target_motor_vel - self.cur_motor_vel
        self.err_steer_ang = self.target_steer_ang - self.cur_steer_ang

    def eStop_callback(self, data):
        if data.data:
            self.target_steer_ang = 0.0
            self.target_motor_vel = 0.0

    def dynamics_update(self, delta_t):
        # steering
        err_steer_vel = -self.err_steer_ang/self.tau_steer
        self.err_steer_ang += err_steer_vel * delta_t
        self.cur_steer_ang = self.target_steer_ang - self.err_steer_ang
        tan_steer = np.tan(self.cur_steer_ang)
        steer_ang_l = np.arctan2(self.body_length*tan_steer, self.body_length - 0.5*self.body_width*tan_steer)
        steer_ang_r = np.arctan2(self.body_length*tan_steer, self.body_length + 0.5*self.body_width*tan_steer)

        # velocity
        err_motor_acc = -self.err_motor_vel/self.tau_motor
        self.err_motor_vel += err_motor_acc * delta_t
        self.cur_motor_vel = np.clip(self.target_motor_vel - self.err_motor_vel, 0.0, np.inf)
        wheel_vel = self.cur_motor_vel/self.gear_ratio
        temp = self.body_width*tan_steer/(2.0*self.body_length)
        wheel_vel_rl = wheel_vel*(1.0 - temp)
        wheel_vel_rr = wheel_vel*(1.0 + temp)
        wheel_vel_fl = wheel_vel*np.sqrt((1.0 - temp)**2 + tan_steer**2)
        wheel_vel_fr = wheel_vel*np.sqrt((1.0 + temp)**2 + tan_steer**2)

        return steer_ang_l, steer_ang_r, wheel_vel_rl, wheel_vel_rr, wheel_vel_fl, wheel_vel_fr

    def timer_thread(self):
        while self.is_timer_run:
            # publish gazebo data
            steer_ang_l, steer_ang_r, wheel_vel_rl, wheel_vel_rr, wheel_vel_fl, wheel_vel_fr \
                                                    = self.dynamics_update(self.timer_period)
            self.pub_pos_left_steering_hinge.publish(steer_ang_l)
            self.pub_pos_right_steering_hinge.publish(steer_ang_r)
            self.pub_vel_left_rear_wheel.publish(wheel_vel_rl)
            self.pub_vel_right_rear_wheel.publish(wheel_vel_rr)
            self.pub_vel_left_front_wheel.publish(wheel_vel_fl)
            self.pub_vel_right_front_wheel.publish(wheel_vel_fr)

            # publish motor angle data
            self.pub_ang_data.publish(self.cur_motor_ang)
            self.pub_ang_vel_data.publish(self.cur_motor_vel)

            # sleep
            self.timer_rate.sleep()


if __name__ == '__main__':
    race_car = RaceCar()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    race_car.is_timer_run = False
    race_car.timer_thread_id.join()
