from visualization_msgs.msg import Marker
import rospy

class Detector(object):

    def __init__(self):
        self.found = False
        rospy.init_node("detect_node")
        rospy.loginfo("Victim Founds")
        rospy.Subscriber('/visualization_marker', Marker, self.detect_callback)
        rospy.spin()


    def detect_callback(self, msg):
        self.found = True
        rospy.loginfo("Victim Found")

#if __main
