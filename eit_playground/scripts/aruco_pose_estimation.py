#!/usr/bin/env python3
# -*- coding: utf-8 -*-

###############################################
# Standard Imports                            #
###############################################
import threading
import cv2
import time
from cv_bridge import CvBridge
from aruco_marker import ArucoMarker
import numpy as np

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

        self.bridge = CvBridge()

        # set to the size of the real marker 
        self.markerLength = 0.1

        self.current_image = np.zeros([480, 640, 3])
        self.frame = 0
        self.rotationVector = 0
        self.translationVector = 0
        self.ids = 0

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
            self.frame, self.rotationVector, self.translationVector, self.ids = self.arucoMarker.tutorial_03_aruco_marker_pose_estimation(self.current_image.astype('uint8'), self.markerLength)
            print(self.translationVector)

    def show_image(self):
        while not rospy.is_shutdown():
                cv2.imshow('Drone camera', self.frame)
                cv2.waitKey(1) 
        cv2.destroyAllWindows() 

if __name__ == '__main__':
    SPC = ArucoPoseEstimatorNode()