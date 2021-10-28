import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge
import cv2

class VideoPublisher:
    vid_pub = None
    bridge = None
    rate = None
    def __init__(self):
        self.video_pub = rospy.Publisher('mono_cam/image_raw', Image, queue_size=10)
        rospy.init_node("video_publisher")
        self.rate = rospy.Rate(10) #10 hz
        # Convert between ROS and OpenCV images
        self.bridge = CvBridge()

    def publish_video_feed(self):
        # open camera
        cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)

        # set dimensions
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        while not rospy.is_shutdown():
            # take frame
            ret, frame = cap.read()
            if(ret == True):
                rospy.loginfo("Publish video frame...")
                self.video_pub.publish(self.bridge.cv2_to_imgmsg(frame))
        # write frame to file
        cv2.imwrite('image.jpg', frame)
        # release camera
        cap.release()

if __name__ == '__main__':
    VideoPublisher().publish_video_feed()