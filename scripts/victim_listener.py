#!/usr/bin/env python
"""This is a simple node that listens for messages on the /victim topic
and republishes the image componenet as an image and the location component
as a marker.

This node also creates a csv file containing all of the final victim
information that was received as well as saved version of the victim
images.

Subscribes to:
   /victim

Publishes to:
   /victim_image
   /victim_marker

Author: Nathan Sprague
Version:  11/2016

"""
import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError

from zeta_rescue.msg import Victim
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

def make_sphere_marker(point, header,
                       ident=0, color=(1.,0.,0.), scale=.5):
    """ Create a sphere marker message at the indicated position. """
    marker = Marker()
    marker.header = header
    marker.ns = 'spheres'
    marker.id = ident
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.pose.position.x = point.x
    marker.pose.position.y = point.y
    marker.pose.position.z = point.z
    marker.scale.x = marker.scale.y = marker.scale.z = scale    
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration(0) # forever
    return marker


class VictimListener(object):
    """
    """

    def __init__(self, file_prefix):
        """
        Set up the node.  There is no main loop, all publishing
        will happen in response to scan callbacks.
        """
        rospy.init_node('victim_listener')
        rospy.Subscriber('/victim', Victim, self.victim_callback,
                         queue_size=10)
        self.image_pub = rospy.Publisher('/victim_image', Image, queue_size=10)
        self.marker_pub = rospy.Publisher('/victim_marker', Marker,
                                            queue_size=10)


        cv_bridge = CvBridge()

        self.victims = {}

        rospy.spin()

        rospy.loginfo("Saving victim data...")
        outfile = open(file_prefix + ".csv", 'w')
        for victim_id in self.victims:
            victim = self.victims[victim_id]
            cv2_img = cv_bridge.imgmsg_to_cv2(victim.image, "bgr8")
            img_name = "{}{}.jpg".format(file_prefix, victim_id)
            cv2.imwrite(img_name, cv2_img)
            line = '{}, {}, "{}", "{}"\n'.format(victim.point.x,
                                                 victim.point.y,
                                                 img_name,
                                                 victim.description)
            outfile.write(line)
        outfile.close()
        rospy.loginfo("Done.")

    def victim_callback(self, victim):
        """ Republish the victim message and store the victim info
        in the dictionary. """
        
        self.victims[victim.id] = victim
        point_stamped = PointStamped()
        point_stamped.header = victim.header
        point_stamped.point = victim.point
        self.image_pub.publish(victim.image)
        marker = make_sphere_marker(victim.point, victim.header,
                                    victim.id, (1., .65, 0))
        self.marker_pub.publish(marker)



if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Provide a prefix for the file..." + sys.argv[2])
    else:
        VictimListener(sys.argv[1])

