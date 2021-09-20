#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
offboard_ctrl.py: Controlling the setpoints, used for testing the OptiTrack system

"""

###############################################
# Standard Imports                            #
###############################################
import time
import threading
from math import *
import numpy as np

###############################################
# ROS Imports                                 #
###############################################
import rospy
import rospkg

###############################################
# ROS Topic messages                          #
###############################################
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State

###############################################
# ROS Service messages                        #
###############################################
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

###############################################
# Offboad Control class                       #
###############################################
class OffboardControl:
    def __init__(self, *args):
        self.current_state = State()
        #self.curr_global_pos = NavSatFix()

        rospy.init_node('spc_server')

        self.offb_set_mode = SetMode

        self.prev_state = self.current_state

        # Publishers
        self.local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # Subscribers
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.cb_state)
        self.sub_mocap_target = rospy.Subscriber('/mavros/offbctrl/target', PoseStamped, self.cb_mocap_target)

        # Services
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        ## Create services
        self.setpoint_controller_server()

        # Init msgs
        self.target = PoseStamped()
        self.target.pose.position.x = 0
        self.target.pose.position.y = 0
        self.target.pose.position.z = 1

        self.last_request = rospy.get_rostime()
        self.state = "INIT"

        self.rate = rospy.Rate(20.0) # MUST be more then 2Hz

        self.t_run = threading.Thread(target=self.navigate)
        self.t_run.start()
        print(">> SetPoint controller is running (Thread)")

        # Spin until the node is stopped
        rospy.spin()

    """
    Callbacks
    * cb_state: Get current state
    * cb_target: Set target positiong (from external via /mavros/offbctrl/target topic)
    """
    def cb_state(self,state):
        self.current_state = state

    def cb_mocap_target(self,data):
        self.set_target(data)

    """
    Services
    * s_stop: Stop all setpoint controllers
    * s_circle: Start Circle setpoint controller
    """
    def setpoint_controller_server(self):
        s_stop = rospy.Service('setpoint_controller/stop', Empty, self.stop)
        s_circle = rospy.Service('setpoint_controller/circle', Empty, self.start_circle)

        print("The SetPoint Controller is ready")

    """
    State
    * set_state: Set current state
    * get_state: Get current state
    """
    def set_state(self, data):
        self.state = data
        print("New State: {}".format(data))

    def get_state(self):
        return self.state

    """
    Target position
    * set_target: Set target position (data format: PoseStamped)
    * set_target_xyz: Set target position (data format: x,y,z,delay)
    * get_target: Get target position
    """
    def set_target(self, data):
        self.target = data

    def set_target_xyz(self,x,y,z):

        print(">> New setpoint: {} {} {}".format(x,y,z))

        self.target.pose.position.x = x
        self.target.pose.position.y = y
        self.target.pose.position.z = z

    def get_target(self):
        return self.target

    """
    Commands
    """
    def navigate(self):
        self.set_state("RUNNING")
        while not rospy.is_shutdown():
            self.target.header.frame_id = "base_footprint"
            self.target.header.stamp = rospy.Time.now()

            self.local_pos_pub.publish(self.target)
            self.rate.sleep()

        print(">> Navigation thread has stopped...")
        self.set_state("STOPPED")

    """
    Circle
    """
    def start_circle(self, r):
        self.t_circle = threading.Thread(target=self.circle)
        self.t_circle.start()
        print(">> Starting circle (Thread)")

        return {}

    def circle(self):
        sides = 360
        radius = 1
        i = 0
        delay = 0.5

        if(self.state != "RUNNING"):
            print(">> SetPoint controller is not running...")
        else:
            self.set_state("CIRCLE")

            while self.state == "CIRCLE":
                x = radius * cos(i*2*pi/sides)
                y = radius * sin(i*2*pi/sides)
                z = self.target.pose.position.z

                self.set_target_xyz(x,y,z)
                time.sleep(delay)

                i = i + 1

                # Reset counter
                if(i > 360):
                    i = 0

            self.set_state("RUNNING")


    """
    Stop
    """
    def stop(self,r):
        self.set_state("STOP")
        return {}

if __name__ == '__main__':
    SPC = OffboardControl()
