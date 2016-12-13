import rospy
import cv2
import time
import map_utils
import actionlib
import random
import tf
import detect
import math

from cv_bridge import CvBridge,CvBridgeError
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Image
from zeta_rescue.msg import Victim

class Detector_Alvar(object):

    def __init__(self):
        rospy.init_node("detect_node")

        self.finding = True
        self.listOfVictims = []
        self.bridge = CvBridge()
        self.map_msg = None
        self.imgCounter = 1
        self.found = False
        self.victim = Victim()
        self.thresh_hold = 1
        self.victim_hold = .1
        self.curX = 2.86     #hardcoded based off of current intital_pose.yaml
        self.curY = -1.77     #hardcoded based off of current intital_pose.yaml
        self.listOfPoints = [(self.curX, self.curY)]
        self.first = True
        self.xpostive = 1
        self.ypostive = 1

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
        
        self.pub = rospy.Publisher('victim', Victim, queue_size=10)
        rospy.Subscriber('map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/camera/rgb/image_raw',Image,self.icallback)
        rospy.Subscriber('/visualization_marker', Marker, self.detect_callback)

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
            print "try random point: ", x_target, ", ", y_target
            if self.map.get_cell(x_target, y_target) == 0:
                if self.checkPoint(x_target,y_target):
                    print "point was valid. going to", x_target,", ", y_target
                    self.count += 1
                    self.goto_point(x_target, y_target)

    def detect_callback(self, msg):
        if self.finding:
            self.finding = False
            if newVictim(msg):
                self.ac.cancel_all_goals()
                #get current x and y and pass them into addpoints
                #self.goto_point(infront of victim with .9 distance away)
                #self.goto_point(where robot was before going to infront of victim)
            else:
                self.finding = True

        if msg.pose.position.z <= .9: ######stuff in here might need to be redone to fit current logic not sure as of now
            self.found = True
            self.victim.point = msg.pose.position
            self.victim.id += 1
            rospy.loginfo("Victim Found")
            rospy.loginfo(self.victim)
            self.pub.publish(self.victim)
            #rospy.loginfo(msg.pose.position)

    def map_callback(self, map_msg):
        """ map_msg will be of type OccupancyGrid """
        self.map_msg = map_msg

    def icallback(self,img):
        if self.imgCounter < 50 and self.found is True:
            try:
                self.found = False
                self.victim.image = img
                self.pub.publish(self.victim)               
                cv_image = self.bridge.imgmsg_to_cv2(img,"rgb8")
                #cv.SaveImage(str(imgCounter) + "image.jpg",cv_image)
                cv2.imwrite(str(self.imgCounter) + "image.jpg",cv_image)
                self.imgCounter = self.imgCounter + 1
                print "image made"
            except CvBridgeError, e:
                print "bye"
                print e

    def goal_message(self, x_target, y_target, theta_target):
        """ Create a goal message in the base_link coordinate frame"""

        quat = tf.transformations.quaternion_from_euler(0, 0, theta_target)
        # Create a goal message ...
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
        ########## i believe there should be a addPoints call here but im not sure on the logic of the rest of the code and ill need to test the cancel call to make sure it doesnt add the points twice

    def newVictim(self,msg):####################  the x will fail here but you can get the x from the msg.pose.x or whatever 
        #will check if the position is a new victim
        if len(self.listOfVictims) != 0:
            for point in self.listOfVictims:
                if (point[0] + self.victim_hold <= x - self.victim_hold or point[0] - self.victim_hold >= x + self.victim_hold) or (point[1] + self.victim_hold <= y - self.victim_hold or point[1] - self.victim_hold >= y + self.thresh_hold):
                    newVictim = True
                else:
                    newVictim = False
                    break
        else:
            return True
        if newVictim:
            #add new victim x and y to victim list
            return True
        else:
            return False

    def checkPoint(self,x,y):
        for point in self.listOfPoints:
            #if point is too close to an already-traversed point
            if (point[0] + self.thresh_hold <= x - self.thresh_hold or point[0] - self.thresh_hold >= x + self.thresh_hold) or (point[1] + self.thresh_hold <= y - self.thresh_hold or point[1] - self.thresh_hold >= y + self.thresh_hold):
                self.goodToGo = True
            else:
                self.goodToGo = False
                break

        return self.goodToGo

    def addPoints(self,x,y):
        slope = 0
        self.checkPosOrNeg(x,y)
        if x-self.curX != 0:
            slope = (y-self.curY)/(x-self.curX)
            intercept = y - (slope * x)
            self.listOfPoints.append((x+(1*self.xpositive),(x+(1*self.xpositive))*slope+intercept))
            for val in range(1,int(math.floor(abs(x-self.curX))+1)):
                self.listOfPoints.append((self.curX+(val*self.xpositive),(self.curX+(val*self.xpositive))*slope+intercept))
        else:
            self.listOfPoints.append((x,y+(1*self.ypositive)))
            for val in range(1,int(math.floor(abs(y-self.curY))+1)):
                self.listOfPoints.append((self.curX,self.curY + (val*self.ypositive)))

        self.curX = x
        self.curY = y

    def checkPosOrNeg(self,x,y):
        if self.curX - x > 0:
            self.xpositive = -1
        else:
            self.xpositive = 1

        if self.curY - y > 0:
            self.ypositive = -1
        else:
            self.ypositive = 1

if __name__ == "__main__":
    Detector_Alvar()
