###############################################
# ROS Imports                                 #
###############################################
import rospy
import rospkg

###############################################
# ROS Service messages                        #
###############################################
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


class SimAttachDetach:
    def __init__(self, *args) :
        
        self.base_attach_client = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.base_detach_client = rospy.ServiceProxy('/link_attacher_node/detach', Attach)

        self.base_attach_client.wait_for_service()
        self.base_detach_client.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach and /link_attacher_node/detach")

        self.att_req = AttachRequest()
        self.att_req.model_name_1 = "QuickConnectBase"
        self.att_req.link_name_1 = "base_link"
        self.att_req.model_name_2 = "sdu_drone_mono_cam_downward"
        self.att_req.link_name_2 = "base_link"

    def attach(self):
        self.base_attach_client.call(self.att_req)
        rospy.loginfo("Simulation drone connected to base")

    def detach(self):
        self.base_detach_client.call(self.att_req)
        rospy.loginfo("Simulation drone disconnected from base")