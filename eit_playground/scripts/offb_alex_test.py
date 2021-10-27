#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
offboard_ctrl.py: Controlling the setpoints

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

        rospy.init_node('offboard_ctrl')

        self.offb_set_mode = SetMode

        self.prev_state = self.current_state

        # Publishers
        self.local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # Subscribers
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.cb_state)
        self.sub_target = rospy.Subscriber('/mavros/offbctrl/target', PoseStamped, self.cb_target)
        self.sub_pose = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.cb_position)
        

        # Services
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        #self.land_client = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

        #self.land_cmd = CommandTOL()
        #self.land_cmd.min_pitch = 0
        #self.land_cmd.yaw = 0
        #self.land_cmd.latitude = 0
        #self.land_cmd.longitude = 0
        #self.land_cmd.altitude = 0

        ## Create services
        self.setpoint_controller_server()

        # Init msgs
        self.target = PoseStamped()
        self.target.pose.position.x = 0
        self.target.pose.position.y = 0
        self.target.pose.position.z = 0

        # Init current position variables

        self.positionX = 0.0
        self.positionY = 0.0
        self.positionZ = 0.0

        # Set to false when testing for real. Deactivates auto arming. 
        self.sim = True

        self.wp_list = [[0, 0, 5], [1.5, 0, 2], [0, 0, 2]]
        self.wp_index = 0

        self.wp_radius = 0.2


        self.last_request = rospy.get_rostime()
        self.state = "INIT"

        self.rate = rospy.Rate(20.0) # MUST be more then 2Hz

        self.t_run = threading.Thread(target=self.navigate)
        self.t_run.start()

        self.set_target_xyz(0,0,0, 0)
        
        self.wp_run = threading.Thread(target=self.check_wp)
        self.wp_run.start()
        print(">> SetPoint controller is running (Thread)")

        # Spin until the node is stopped

        time.sleep(5)
        tmp = Empty()
        self.switch2offboard(tmp)
    
        rospy.spin()

    """
    Callbacks
    * cb_state
    * cb_target
    """
    def cb_state(self,state):
        self.current_state = state

    def cb_target(self,data):
        self.set_target(data)

    def cb_position(self, ps):
        self.positionX = ps.pose.position.x
        self.positionY = ps.pose.position.y
        self.positionZ = ps.pose.position.z      

    """
    Services
    * s_arm:
    * s_stop:
    * s_s2o:
    """
    def setpoint_controller_server(self):
        s_arm = rospy.Service('setpoint_controller/arm', Empty, self.arm)
        s_stop = rospy.Service('setpoint_controller/stop', Empty, self.stop)
        s_s2o = rospy.Service('setpoint_controller/switch2offboard', Empty, self.switch2offboard)

        print("The SetPoint Controller is ready")

    """
    State
    * set_state:
    * get_state:
    """
    def set_state(self, data):
        self.state = data
        print("New State: {}".format(data))

    def get_state(self):
        return self.state

    """
    Target position
    * set_target:
    """

    def check_wp(self):
        while not rospy.is_shutdown():
            if(abs(self.positionX - self.target.pose.position.x) < self.wp_radius and abs(self.positionY - self.target.pose.position.y) < self.wp_radius and abs(self.positionZ - self.target.pose.position.z) < self.wp_radius):
                if self.wp_index < len(self.wp_list):
                    print("reached waypoint: ", self.wp_list[self.wp_index])
                    self.set_target_xyz(self.wp_list[self.wp_index][0],self.wp_list[self.wp_index][1], self.wp_list[self.wp_index][2], 0)
                    self.wp_index += 1
                else:
                    self.set_mode_client(base_mode=0, custom_mode="AUTO.LAND")
                    #self.land_client.call(self.land_cmd.min_pitch, self.land_cmd.yaw, self.land_cmd.latitude, self.land_cmd.longitude, self.land_cmd.altitude)
            self.rate.sleep()

    def set_target(self, data):
        self.target = data

    def set_target_xyz(self,x,y,z,delay):

        if(delay > 0.1):
            print(">> New setpoint: {} {} {}".format(x,y,z))

        self.target.pose.position.x = x
        self.target.pose.position.y = y
        self.target.pose.position.z = z

        time.sleep(delay)

    def get_target(self):
        return self.target

    """
    Commands
    * switch2offboard:
    * arm:
    * navigate:
    * stop: 
    """
    def switch2offboard(self,r):
        print(">> Starting OFFBOARD mode")

        last_request = rospy.get_rostime()
        while self.current_state.mode != "OFFBOARD":
            now = rospy.get_rostime()
            if(now - last_request > rospy.Duration(5)):
                print("Trying: OFFBOARD mode")
                self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                last_request = now

        tmp = Empty()
        self.arm(tmp)

        return {}

    def arm(self,r):
        print(">> Arming...")
        last_request = rospy.get_rostime()
        while not self.current_state.armed:
            now = rospy.get_rostime()
            if(now - last_request > rospy.Duration(5.)):
               if self.sim: 
                    self.arming_client(True)
               last_request = now

        return {}

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
    Stop
    """
    def stop(self,r):
        self.set_state("STOP")
        return {}

if __name__ == '__main__':
    SPC = OffboardControl()
    