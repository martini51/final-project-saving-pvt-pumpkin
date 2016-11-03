#!/usr/bin/env python

"""Publishes an empty message to the "report_requested" topic
whenever the button is pressed.  Code borrowed from:

http://effbot.org/tkinterbook/

Author: Nathan Sprague
Version: 10/15

"""

from Tkinter import *
import rospy
from std_msgs.msg import Empty

class ReportButton(object):
    """ Node that presents a tkInter button. """

    def __init__(self):
        """ Set up the button and the callback. """
        rospy.init_node("button_node")
        self.pub = rospy.Publisher('report_requested', Empty, queue_size=10)
        master = Tk()
        f = Frame(master, height=100, width=240)
        f.pack_propagate(0) # don't shrink
        f.pack()

        b = Button(f, text="CLICK FOR REPORT", command=self.callback)
        b.pack(fill=BOTH, expand=1)

        mainloop()


    def callback(self):
        rospy.loginfo("Report Requested.")
        self.pub.publish(Empty())

if __name__ == "__main__":
    ReportButton()
