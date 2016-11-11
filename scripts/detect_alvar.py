from visualization_msgs.msg import Marker
import rospy
import time
import map_utils
import actionlib
import random
import tf

from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point
from actionlib_msgs.msg import GoalStatus

class Detector(object):

    def __init__(self):
        rospy.init_node("detect_node")

        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.state_names = {}
        self.state_names[GoalStatus.PENDING] = "PENDING"
        self.state_names[GoalStatus.ACTIVE] = "ACTIVE"
        self.state_names[GoalStatus.PREEMPTED] = "PREEMPTED"
        self.state_names[GoalStatus.SUCCEEDED] = "SUCCEEDED"
        self.state_names[GoalStatus.ABORTED] = "ABORTED"
        self.state_names[GoalStatus.REJECTED] = "REJECTED"
        self.state_names[GoalStatus.RECALLED] = "RECALLED"
        self.state_names[GoalStatus.LOST] = "LOST"
        
        rospy.Subscriber('map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/visualization_marker', Marker, self.detect_callback)


        self.map_msg = None

        while self.map_msg is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for map...")
            rospy.sleep(.1)

        self.map = map_utils.Map(self.map_msg)

        self.count = 0

        x_target = 0
        y_target = 0
        while not rospy.is_shutdown() and self.count < 3:
            #while not self.map.get_cell(x_target, y_target) == 0:

            x_target = random.uniform(-10,10)
            y_target = random.uniform(-10,10)

            if self.map.get_cell(x_target, y_target) == 0:
                self.count += 1
                self.goto_point(x_target, y_target)

#	        rospy.spin()


    def detect_callback(self, msg):
        rospy.loginfo("Victim Found")
        rospy.loginfo(msg)

    def map_callback(self, map_msg):
        """ map_msg will be of type OccupancyGrid """
        self.map_msg = map_msg

    def goal_message(self, x_target, y_target, theta_target):
        """ Create a goal message in the base_link coordinate frame"""

        quat = tf.transformations.quaternion_from_euler(0, 0, theta_target)
        # Create a goal message ...
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.pose.position.x = x_target
        goal.target_pose.pose.position.y = y_target
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]
        return goal

    def goto_point(self, x_target, y_target, theta_target=0):

        """ Move to a location relative to the robot's current position """

        rospy.loginfo("navigating to: ({},{})".format(x_target, y_target))

        goal = self.goal_message(x_target, y_target, theta_target)

        rospy.loginfo("Waiting for server.")
        self.ac.wait_for_server()

        rospy.loginfo("Sending goal.")
        self.ac.send_goal(goal)
        rospy.loginfo("Goal Sent.")

        # Check in after a while to see how things are going.
        rospy.sleep(1.0)
        rospy.loginfo("Status Text: {}".format(self.ac.get_goal_status_text()))

        # Should be either "ACTIVE"
        state_name = self.state_names[self.ac.get_state()]
        rospy.loginfo("State      : {}".format(state_name))

        # Wait until the server reports a result.
        self.ac.wait_for_result()
        rospy.loginfo("Result Text: {}".format(self.ac.get_goal_status_text()))

        # Should be either "SUCCEEDED" or "ABORTED"
        state_name = self.state_names[self.ac.get_state()]
        rospy.loginfo("State      : {}".format(state_name))
        self.count = self.count + 1

if __name__ == "__main__":
    Detector()
