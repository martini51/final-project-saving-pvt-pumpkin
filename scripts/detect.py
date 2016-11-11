from visualization_msgs.msg import Marker
import rospy

class Detector(object):

    def __init__(self):
        rospy.init_node("detect_node")
        rospy.Subscriber('/visualization_marker', Marker, self.detect_callback)
        rospy.spin()


    def detect_callback(self, msg):
        rospy.loginfo("Victim Found")

if __name__ == "__main__":
    Detector()
