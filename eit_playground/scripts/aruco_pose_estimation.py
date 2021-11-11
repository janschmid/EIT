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

        self.cam_sub = rospy.Subscriber('/mono_cam/image_raw', Image, self.cb_image)

        self.aruco_pos_pub = rospy.Publisher('/aruco_pose', PoseStamped, queue_size=10)

        self.bridge = CvBridge()

        # set to the size of the real marker 
        self.markerLength = 0.1

        self.aruco_pose_msg = PoseStamped()
        self.aruco_pose_msg.pose.position.x = 0
        self.aruco_pose_msg.pose.position.y = 0
        self.aruco_pose_msg.pose.position.z = 0

        self.current_image = np.zeros([480, 640, 3])
        self.frame = 0
        self.rotationVector = 0
        self.translationVector = 0
        
        if(rospy.get_param("SIMULATION")==True):
            self.markerPostfixName = "simulation"
        else:
            self.markerPostfixName = "piCam"
        rospy.loginfo("Using {0} calibration".format(self.markerPostfixName))
        #self.ids = 

        self.t_show_image = threading.Thread(target=self.show_image)
        self.t_show_image.start()

        self.arucoMarker = ArucoMarker()

        self.t_pose_estimation = threading.Thread(target=self.estimate_pose)
        self.t_pose_estimation.start()

        rospy.spin()

    def cb_image(self, image):
        self.current_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')


    def estimate_pose(self):
        while not rospy.is_shutdown():
            self.frame, self.rotationVector, self.translationVector, self.ids = self.arucoMarker.tutorial_03_aruco_marker_pose_estimation(
                self.current_image.astype('uint8'), self.markerLength, self.markerPostfixName)
            #for x in range(len(self.ids)):
            #print(self.ids)
            if self.ids is not None: 
                #print(self.translationVector[0][0][0][0])
                q = quaternion_from_euler(self.rotationVector[0][0][0][0], self.rotationVector[0][0][0][1], self.rotationVector[0][0][0][2])
                #print(q)
                self.aruco_pose_msg.header.frame_id = "aruco_marker"
                self.aruco_pose_msg.header.stamp = rospy.Time.now()
                self.aruco_pose_msg.pose.position.x = self.translationVector[0][0][0][0]
                self.aruco_pose_msg.pose.position.y = self.translationVector[0][0][0][1]
                self.aruco_pose_msg.pose.position.z = self.translationVector[0][0][0][2]
                self.aruco_pose_msg.pose.orientation.x = q[0]
                self.aruco_pose_msg.pose.orientation.y = q[1]
                self.aruco_pose_msg.pose.orientation.z = q[2]
                self.aruco_pose_msg.pose.orientation.w = q[3]

                self.aruco_pos_pub.publish(self.aruco_pose_msg)
            else:
                self.aruco_pose_msg.pose.position.x = 0.0
                self.aruco_pose_msg.pose.position.y = 0.0
                self.aruco_pose_msg.pose.position.z = 0.0 
                self.aruco_pos_pub.publish(self.aruco_pose_msg)

            else:
                self.aruco_pose_msg.header.frame_id = "aruco_marker"
                self.aruco_pose_msg.header.stamp = rospy.Time.now()
                self.aruco_pose_msg.pose.position.x = 0
                self.aruco_pose_msg.pose.position.y = 0
                self.aruco_pose_msg.pose.position.z = 0

                self.aruco_pos_pub.publish(self.aruco_pose_msg)


    def show_image(self):
        if(rospy.get_param("SIMULATION")==True):
            while not rospy.is_shutdown():
                    cv2.imshow('Drone camera', self.frame)
                    cv2.waitKey(1) 
            cv2.destroyAllWindows() 

if __name__ == '__main__':
    SPC = ArucoPoseEstimatorNode()
