#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan, JointState

import numpy as np
import threading
import time

# subscribe values
target_steer = 0.0
target_vel = 0.0
cur_vel = 0.0

# kinematic & dynamics
WB = 0.5 #0.325
TW = 0.2
gamma = 0.32
omega = 1.6
gear_ratio = 55.0/19.0

def drive_pub_thread(pub_vel_left_rear_wheel, 
                    pub_vel_right_rear_wheel,
                    pub_vel_left_front_wheel,
                    pub_vel_right_front_wheel,
                    pub_pos_left_steering_hinge,
                    pub_pos_right_steering_hinge):
    global target_vel, target_steer, WB, TW, gamma, omega
    rate = rospy.Rate(1/drive_pub_thread.period)

    while drive_pub_thread.is_run:
        tan_steer = np.tan(target_steer)
        steer_left = np.arctan2(WB*tan_steer, WB - 0.5*TW*tan_steer)
        steer_right = np.arctan2(WB*tan_steer, WB + 0.5*TW*tan_steer)

        drive_pub_thread.error_dot = -2.0*gamma*drive_pub_thread.error - (omega*omega + gamma*gamma)*drive_pub_thread.error_integ
        drive_pub_thread.error += drive_pub_thread.error_dot*drive_pub_thread.period
        drive_pub_thread.error_integ*= 0.99
        drive_pub_thread.error_integ += drive_pub_thread.error*drive_pub_thread.period
        vel = np.clip(target_vel - drive_pub_thread.error, 0.0, np.inf)
        #rospy.logwarn("[servo_commands] error : {:.5f}, error integral : {:.5f}, vel : {:.5f}".format(drive_pub_thread.error, drive_pub_thread.error_integ, vel))

        vel /= gear_ratio
        temp = TW*tan_steer/(2.0*WB)
        vel_rl = vel*(1.0 - temp)
        vel_rr = vel*(1.0 + temp)
        vel_fl = vel*np.sqrt((1.0 - temp)**2 + tan_steer**2)
        vel_fr = vel*np.sqrt((1.0 + temp)**2 + tan_steer**2)

        pub_vel_left_rear_wheel.publish(vel_rl)
        pub_vel_right_rear_wheel.publish(vel_rr)
        pub_vel_left_front_wheel.publish(vel_fl)
        pub_vel_right_front_wheel.publish(vel_fr)
        pub_pos_left_steering_hinge.publish(steer_left)
        pub_pos_right_steering_hinge.publish(steer_right)
        
        rate.sleep()

drive_pub_thread.is_run = True
drive_pub_thread.period = 0.01
drive_pub_thread.error = 0.0
drive_pub_thread.error_integ = 0.0
drive_pub_thread.error_dot = 0.0


def rpm_callback(data):
    global cur_vel, gear_ratio
    idx1 = data.name.index('left_rear_wheel_joint')
    idx2 = data.name.index('right_rear_wheel_joint')
    cur_vel = (data.velocity[idx1] + data.velocity[idx2])/2
    cur_vel *= gear_ratio

def set_throttle_steer(data):
    global target_steer, target_vel
    target_steer = data.drive.steering_angle
    target_vel = data.drive.speed
    drive_pub_thread.error = target_vel - cur_vel

def servo_commands():
    rospy.init_node('servo_commands', anonymous=True)

    rospy.Subscriber("/racecar/ackermann_cmd_mux/output", AckermannDriveStamped, set_throttle_steer)
    rospy.Subscriber('/racecar/joint_states', JointState, rpm_callback)
    pub_vel_left_rear_wheel = rospy.Publisher('/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher('/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_left_front_wheel = rospy.Publisher('/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher('/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_pos_left_steering_hinge = rospy.Publisher('/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)

    drive_thread = threading.Thread(target=drive_pub_thread, args=(pub_vel_left_rear_wheel, 
                                                                pub_vel_right_rear_wheel,
                                                                pub_vel_left_front_wheel,
                                                                pub_vel_right_front_wheel,
                                                                pub_pos_left_steering_hinge,
                                                                pub_pos_right_steering_hinge))
    drive_thread.daemon=True
    drive_thread.start()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    drive_pub_thread.is_run = False

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
