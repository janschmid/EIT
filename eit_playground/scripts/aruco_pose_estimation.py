#!/usr/bin/env python3
# -*- coding: utf-8 -*-

###############################################
# Standard Imports                            #
###############################################
import threading
import cv2
import time
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from aruco_marker import ArucoMarker
import numpy as np
import math

from tf.transformations import quaternion_from_euler

###############################################
# ROS Imports                                 #
###############################################
import rospy
import rospkg

###############################################
# ROS Topic messages                          #
###############################################
from sensor_msgs.msg import Image

###############################################
# Camera Subscriber Class                     #
###############################################
class ArucoPoseEstimatorNode:
    def __init__(self, *args):

        rospy.init_node('aruco_pose_estimator')

        self.aruco_pos_pub = rospy.Publisher('/aruco_pose', PoseStamped, queue_size=1)

        self.aruco_pose_msg = PoseStamped()
        self.aruco_pose_msg.pose.position.x = 0
        self.aruco_pose_msg.pose.position.y = 0
        self.aruco_pose_msg.pose.position.z = 0
        self.rate = rospy.Rate(20)
        self.current_image = np.zeros([480, 640, 3])
        self.frame = 0
        
        if(rospy.get_param("SIMULATION")==True):
            self.markerPostfixName = "simulation"
            self.markerLength=0.05
        else:
            self.markerPostfixName = "piCam-360p"
            # set to the size of the real marker 
            self.markerLength = 0.0495
        rospy.loginfo("Using {0} calibration, marker length is : {1}".format(self.markerPostfixName, self.markerLength))
        #self.ids = 
        if(rospy.get_param("SIMULATION")==True):
            self.t_show_image = threading.Thread(target=self.show_image)
            self.t_show_image.start()

        self.arucoMarker = ArucoMarker()

        self.t_pose_estimation = threading.Thread(target=self.estimate_pose)
        self.t_pose_estimation.start()

        rospy.spin()

    def estimate_pose(self):
        cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)

        # set dimensions
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
        while not rospy.is_shutdown():
            ret, current_image = cap.read()
            if(ret == False):
                rospy.loginfo("Could not access /dev/video0")
                time.sleep(5)
                continue
            self.frame, eulerAngle, corrected_global_position = self.arucoMarker.get_global_pos_and_euler_angles(current_image.astype('uint8'), self.markerLength, self.markerPostfixName)

            if (eulerAngle is not None and
                    corrected_global_position is not None and
                    len(eulerAngle)>0 and len(corrected_global_position)>0):
                self.aruco_pose_msg.header.frame_id = "aruco_marker"
                self.aruco_pose_msg.pose.position.x = corrected_global_position[0]
                self.aruco_pose_msg.pose.position.y = corrected_global_position[1]
                self.aruco_pose_msg.pose.position.z = corrected_global_position[2]
                q = quaternion_from_euler(eulerAngle[0], eulerAngle[1], eulerAngle[2])
                self.aruco_pose_msg.pose.orientation.x = q[0]
                self.aruco_pose_msg.pose.orientation.y = q[1]
                self.aruco_pose_msg.pose.orientation.z = q[2]
                self.aruco_pose_msg.pose.orientation.w = q[3]
                self.aruco_pos_pub.publish(self.aruco_pose_msg)
                #rospy.loginfo("pos: {0}, orientation: {1}".format(corrected_global_position, eulerAngle))
            else:
                self.aruco_pose_msg.header.frame_id = "no_aruco_marker"
                self.aruco_pose_msg.header.stamp = rospy.Time.now()
                self.aruco_pose_msg.pose.position.x = 0
                self.aruco_pose_msg.pose.position.y = 0
                self.aruco_pose_msg.pose.position.z = 0

                self.aruco_pos_pub.publish(self.aruco_pose_msg)
            self.rate.sleep()


    def show_image(self):
        while not rospy.is_shutdown():
                #print("Shape: ", np.shape(self.frame))
                cv2.circle(self.frame,(320,240), 5, (0,0,255), -1)
                cv2.imshow('Drone camera', self.frame)
                cv2.waitKey(1) 
        cv2.destroyAllWindows() 

if __name__ == '__main__':
    SPC = ArucoPoseEstimatorNode()
