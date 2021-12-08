###############################################
# Imports                                     #
###############################################
import rospy
import rospkg
import time
import RPi.GPIO as GPIO

class ConnectorActuation:
    def __init__(self):
        self.connectorOpen = True
        # GPIO.setmode(GPIO.BCM)
        # GPIO.cleanup()
        self.gpio_pin = 10

        # GPIO.setup(self.gpio_pin, GPIO.OUT)
        
    def close_connector(self):
        # GPIO.output(self.gpio_pin, 0)
        rospy.loginfo("Real-world connector closed")

    def open_connector(self):
        # GPIO.output(self.gpio_pin, 1)
        rospy.sleep(0.5)
        # GPIO.output(self.gpio_pin, 0)
        rospy.loginfo("Real-world connector opened")