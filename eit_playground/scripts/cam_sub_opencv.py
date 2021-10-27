#!/usr/bin/env python3
# -*- coding: utf-8 -*-

###############################################
# Standard Imports                            #
###############################################
import threading
import cv2
import time
from cv_bridge import CvBridge

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
class CamSubscriber:
    def __init__(self, *args):

        rospy.init_node('cam_subscriber')

        self.cam_sub = rospy.Subscriber('/mono_cam/image_raw', Image, self.cb_image)

        self.bridge = CvBridge()

        self.current_image = 0

        self.t_show_image = threading.Thread(target=self.show_image)
        self.t_show_image.start()

        rospy.spin()

    def cb_image(self, image):
        self.current_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')

        #print(self.current_image)

    def show_image(self):
        while not rospy.is_shutdown():
            #if self.current_image:
                cv2.imshow('Drone camera', self.current_image)
                #print(type(self.current_image))
                cv2.waitKey(1) 
        cv2.destroyAllWindows() 

if __name__ == '__main__':
    SPC = CamSubscriber()