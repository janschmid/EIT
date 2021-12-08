#!/usr/bin/env python3
# -*- coding: utf-8 -*-
###############################################
# Standard Imports                            #
###############################################
from pickle import TRUE
import time
import threading
from math import *
import numpy as np
import scipy
from scipy.spatial.transform import Rotation 
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

if(rospy.get_param("SIMULATION")):
    from sim_attach_detach import SimAttachDetach
else:
    from connector_actuation import ConnectorActuation


# from mavros_msgs.msg import LandingTarget

###############################################
# ROS Service messages                        #
###############################################
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
# from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

###############################################
# Offboad Control class                       #
###############################################


class offb_landing:
    mission_state = 0
    aruco_visible = False
    def __init__(self, *args):
        self.current_state = State()
        rospy.init_node('offb_landing')
        self.landing_succeeded=False
        # Publishers
        self.local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # Subscribers
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.cb_state)
        self.sub_target = rospy.Subscriber('/mavros/offbctrl/target', PoseStamped, self.cb_target)
        self.sub_pose = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.cb_position)
        self.sub_aruco_pose = rospy.Subscriber("/aruco_pose", PoseStamped, self.aruco_pose)

        # Services
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        # Init msgs
        self.target = PoseStamped()
        self.target.pose.position.x = 0
        self.target.pose.position.y = 0
        self.target.pose.position.z = 0        

        # Init current position variables

        self.positionX = 1.0
        self.positionY = 1.0
        self.positionZ = -1.23456

        self.rate = rospy.Rate(4.0) # MUST be more then 2Hz

        self.t_state_observer = threading.Thread(target =self.landing_controller, daemon=True)
        
        self.landing_state = "CENTER_DRONE"

        # Wait for drone position to be published
        while self.positionZ == -1.23456:
            rospy.loginfo("Waiting for position callback")
            rospy.sleep(1)

        # set safety waypoint at current position at 1m height
        self.set_target_xyz(self.positionX, self.positionY, 1, 0.5)
        print("Start drone - increase altitude: x: {0}, y: {1}".format(self.positionX, self.positionY))

        # start navigation thread which constantly publishes the current target waypoint      
        self.t_run = threading.Thread(target=self.navigate, daemon=True)
        self.t_run.start()

        # Only switch to offboard automatically when in simulation. otherwise for pilot to activate it
        if(rospy.get_param("SIMULATION")):
            self.sim_connector = SimAttachDetach()
            self.sim_connector.detach()
        else:
            self.gpio_connector = ConnectorActuation()
            self.gpio_connector.open_connector()

    def marker_landing_connect_waypoint_disconnect_sequence(self):

        self.arming_sequence()

        # Takeoff to 2m at current position
        while not(self.set_target_xyz(self.positionX, self.positionY ,2, 0.5)):
            pass
        
        # Introduce some rotation to demostrate the alignment working
        if(rospy.get_param("SIMULATION")):
            self.set_target_orient(Rotation.from_euler('xyz', [0,0,90], degrees=True), 2)
        

        #### Flying sequence: ####
         
        # Fly in the direction of the marker
        while not(self.set_target_xyz(1.3,0.7 ,2, 0.5)):
            pass

        # start the landing thread once estimated position of marker is reached  
        self.t_state_observer.start()

        # wait until landing was successful
        while not self.landing_succeeded:
            pass
        self.t_state_observer.join()
        rospy.loginfo("Eagle has landed")

        # Wait for drone to disarm
        while self.current_state.armed:
            rospy.sleep(1)
            rospy.loginfo("Waiting for auto disarm!")

        # Close the connector
        self.connect(True)
        rospy.sleep(1)

        # Arm drone again
        self.arming_sequence()

        # Fly waypoint mission
        while not(self.set_target_xyz(0, 0, 2, 0, 0.2)):
            pass
        while not(self.set_target_xyz(5, 0, 1, 0, 0.2)):
            pass
        while not(self.set_target_xyz(5, 0, 0.2, 0, 0.2)):
            pass

        # Land
        self.set_mode_client(base_mode=0, custom_mode="AUTO.LAND")
        
        # Wait for drone to disarm
        while self.current_state.armed:
            rospy.sleep(1)
            rospy.loginfo("Waiting for auto disarm!")

        # Disconnect payload
        self.connect(False)

        self.arming_sequence()

        while not(self.set_target_xyz(self.positionX, self.positionY, 2, 0.3)):
            pass

        while not(self.set_target_xyz(0, 0, 2, 0, 0.2)):
            pass
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


    """
    State
    * set_state:
    * get_state:
    """

    def set_state(self, data):
        self.state = data
        print("New State: {}".format(data))

    def cb_position(self, ps):
        self.positionX = ps.pose.position.x
        self.positionY = ps.pose.position.y
        self.positionZ = ps.pose.position.z 

        self.orientX = ps.pose.orientation.x
        self.orientY = ps.pose.orientation.y
        self.orientZ = ps.pose.orientation.z
        self.orientW = ps.pose.orientation.w

    def aruco_pose(self, ps):
        if(ps.header.frame_id == "aruco_marker"):
            self.aruco_visible = True
        else:
            self.aruco_visible = False
        self.aruco_posX = ps.pose.position.x
        self.aruco_posY = ps.pose.position.y
        self.aruco_posZ = ps.pose.position.z 
        self.aruco_orientX = ps.pose.orientation.x
        self.aruco_orientY = ps.pose.orientation.y
        self.aruco_orientZ = ps.pose.orientation.z
        self.aruco_orientW = ps.pose.orientation.w

    def set_target_xyz(self,x,y,z,delay, radius_of_acceptance = 0.5):

        # if(delay > 0.1):
        #     print(">> New setpoint: {} {} {}".format(x,y,z))

        self.target.pose.position.x = x
        self.target.pose.position.y = y
        self.target.pose.position.z = z

        time.sleep(delay)
        if(sqrt(pow(self.positionX-self.target.pose.position.x, 2)
                +pow(self.positionY-self.target.pose.position.y, 2)
                +pow(self.positionZ-self.target.pose.position.z, 2))<radius_of_acceptance):
            return True
        else:
            return False

    def set_target_orient(self, rotation, delay, z_rot_acceptance_in_deg=1):
        quat = rotation.as_quat()
        self.target.pose.orientation.x = quat[0]
        self.target.pose.orientation.y = quat[1]
        self.target.pose.orientation.z = quat[2]
        self.target.pose.orientation.w = quat[3]
        rospy.sleep(delay)
        current_rot = Rotation.from_quat([self.orientX, self.orientY, self.orientZ, self.orientW])
        if(current_rot.as_euler('xyz')[2]>z_rot_acceptance_in_deg):
            return False
        else:
            return True

    def navigate(self):
        self.set_state("RUNNING")
        while not rospy.is_shutdown():
            self.target.header.frame_id = "base_footprint"
            self.target.header.stamp = rospy.Time.now()

            self.local_pos_pub.publish(self.target)
            self.rate.sleep()

        print(">> Navigation thread has stopped...")
        self.set_state("STOPPED")

    def calculate_xy_to_aruco_marker(self):
        drone_rot_radian = Rotation.from_quat([self.orientX, self.orientY, 
            self.orientZ, self.orientW]).as_euler('xyz', degrees=False)[2]
        #x and y are flipped...
        ax = self.aruco_posX
        ay = self.aruco_posY
        aruco_x_rotated = ax*cos(drone_rot_radian)+ay*sin(drone_rot_radian)
        aruco_y_rotated = -ax*sin(drone_rot_radian)+ay*cos(drone_rot_radian)
        x = self.positionX-aruco_x_rotated
        y = self.positionY+aruco_y_rotated
        return x,y


    def move_towards_aruco_marker(self, radiusOfAcceptance = 0.2, height=None, heightTolerance = None):
        if not self.aruco_visible:
            return False
        #initial condition
        # rotate local aruco marker pos by drone rotation into global position
        
        z = self.positionZ
        x,y = self.calculate_xy_to_aruco_marker()
        if(height is not None):
            z=height

        self.set_target_xyz(x,y,z, 0.5)
        
        distance_to_marker = self.calculate_2d_distance(self.aruco_posX, self.aruco_posY)
        #rospy.loginfo("moveXY, distance: {0}".format(distance_to_marker))
        if(heightTolerance is None or self.aruco_orientZ-height<heightTolerance):
            if(distance_to_marker<radiusOfAcceptance):
                return True
        
        return False
    
    # def descending(self, target_height = 2, tolerance = 0.2):
    #     if not self.aruco_visible:
    #         return False

    #     if(self.aruco_orientZ < (target_height+tolerance)):
    #         return True
    #     else:
    #         self.set_target_xyz(self.positionX, self.positionY, target_height, 1)
    #         return False
    
    def calculate_rotation_to_marker(self):
        if not self.aruco_visible:
            return False
        r_aruco = Rotation.from_quat([self.aruco_orientX, self.aruco_orientY, 
            self.aruco_orientZ, self.aruco_orientW])
        
        r_drone = Rotation.from_quat([self.orientX, self.orientY, 
            self.orientZ, self.orientW])
        
        r_target = r_aruco.inv()

        # z_current = r_drone.as_euler('xyz', degrees=True)[2]
        # z_aruco = r_aruco.as_euler('xyz', degrees=True)[2]
        # z_difference =  (z_aruco-z_current)
        e_aruco = r_aruco.as_euler('xyz', degrees=True)
        e_drone = r_drone.as_euler('xyz', degrees=True)
        e_target = e_aruco*[1,1,-1]+e_drone
        r_target = Rotation.from_euler('xyz', e_target, degrees=True)
        return r_target

    def align_rotation(self, delay = 0, tolerance_in_degrees = 5):
        r_target = self.calculate_rotation_to_marker()

        if r_target == False:
            return False
        else:
            return self.set_target_orient(r_target, delay, tolerance_in_degrees)


        # difference = 
        # rospy.loginfo("Current rotation: {0}, aruco rotation: {1}, applied: {2}".format(
        #     r_drone.as_euler('xyz', degrees=True)[2], 
        #     r_aruco.as_euler('xyz', degrees=True)[2],
        #     r_target.as_euler('xyz', degrees=True)[2])
        # )


    def calculate_2d_distance(self, x, y):
        return sqrt(x*x  + y*y)

    def arming_sequence(self):
        if(rospy.get_param("SIMULATION")):
            self.switch2offboard()
        else:
            while self.current_state.mode != "OFFBOARD":
                rospy.loginfo("!! Waiting for Offboard !!")
                rospy.sleep(1)

    def switch2offboard(self):
        print(">> Starting OFFBOARD mode")

        last_request = rospy.get_rostime()
        while self.current_state.mode != "OFFBOARD":
            now = rospy.get_rostime()
            if(now - last_request > rospy.Duration(5)):
                print("Trying: OFFBOARD mode")
                self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                last_request = now

        #tmp = Empty()
        self.arm()

        return {}

    def arm(self):
        print(">> Arming...")
        last_request = rospy.get_rostime()
        while not self.current_state.armed:
            now = rospy.get_rostime()
            if(now - last_request > rospy.Duration(5.)):
                if(rospy.get_param("SIMULATION")==True):
                    self.arming_client(True)
                    last_request = now
                else:
                    rospy.loginfo("Please arm drone manually, auto arming disabled outside of simulation")

        return {}

    """
    Pass function with boolean return value 
    """
    def execute_until_aligned(self, min_hits, function,*args):
        hits = 0
        rospy.loginfo("Execute: {0}".format(function.__name__))
        while(hits<min_hits):
            if(function(*args)):
                hits += 1
            else:
                hits = 0
        # rospy.loginfo(self.landing_state)


    def landing_controller(self):
        rospy.loginfo("Begin landing sequence")
        self.last_mission_state = -1
        while not self.landing_succeeded:
            # if(self.landing_succeeded == True):
            #     rospy.sleep(1)
            #     continue
            # self.align_rotation()
            # print(">>Start landing...")
            if(self.last_mission_state==-1):
                #Initalize 
                self.last_mission_state = self.mission_state
                #switch behaviour
            if(self.aruco_visible==True):
                rospy.loginfo("Landing step 1")
                self.execute_until_aligned(5, self.move_towards_aruco_marker, 0.3, 1.5)
                self.execute_until_aligned(5, self.align_rotation, 0.5, 10)

                rospy.loginfo("Landing step 2")
                self.execute_until_aligned(5, self.move_towards_aruco_marker, 0.1, 1)
                self.execute_until_aligned(3, self.align_rotation, 1, 5)
                rospy.loginfo("Landing step 3")

                descendingHeight = 0.6
                while(descendingHeight>=0.2):
                    self.execute_until_aligned(3, self.move_towards_aruco_marker, 0.05, descendingHeight)
                    
                    if (self.align_rotation(1, 3) == False) and descendingHeight < 2:
                        descendingHeight+=0.2
                        rospy.loginfo("Lost marker, increase height. ")
                    else:
                        descendingHeight-=0.2
                    rospy.sleep(1)
                rospy.loginfo("Final landing step")
                self.execute_until_aligned(3, self.move_towards_aruco_marker, 0.015, 0.01)
                #while not (self.move_towards_aruco_marker, 0.01, 0.01):
                #rospy.loginfo("LANDING")
                self.set_mode_client(base_mode=0, custom_mode="AUTO.LAND")
                self.landing_succeeded=True
                rospy.loginfo("Landing was successful.")


                # self.connector.attach()
                # rospy.sleep(1)
                # self.set_target_xyz(0, 0, 3, 3)
                
                #rospy.loginfo("Landing was successful")

            #     # self.last_mission_state = self.mission_state
            
            #self.rate.sleep()
        #print(">>mission state observer thread has stopped, landing disabled....")
        #self.set_state("STOPPED")

    def connect(self, close):
        if close:
            if rospy.get_param("SIMULATION"):
                self.sim_connector.attach()
            else:
                self.gpio_connector.close_connector()
        else:
            if rospy.get_param("SIMULATION"):
                self.sim_connector.detach()
            else:
                self.gpio_connector.open_connector()

if __name__ == '__main__':
    o = offb_landing()
    o.marker_landing_connect_waypoint_disconnect_sequence()
    #o.connect(True)
