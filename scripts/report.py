#!/usr/bin/env python

"""Publishes an empty message to the "report_requested" topic
whenever the button is pressed.  Code borrowed from:

http://effbot.org/tkinterbook/

Author: Nathan Sprague
Version: 10/15

"""

from Tkinter import *
import rospy
from zeta_rescue.msg import Victim
from std_msgs.msg import Empty

class ReportButton(object):
    """ Node that presents a tkInter button. """

    def __init__(self):
        """ Set up the button and the callback. """
        rospy.init_node("button_node")

        self.text = 'default'
        while not rospy.is_shutdown():
            rospy.Subscriber('victim', Victim, self.victim_callback)
            master = Tk()
            f = Frame(master, height=100, width=240)
            f.pack_propagate(0) # don't shrink
            f.pack()
            rospy.sleep(1.0)
            b = Button(f, text=self.text, command=self.callback)

            rospy.loginfo(self.text)

            b.pack(fill=BOTH, expand=1)

            mainloop()


    def callback(self):
    
    def victim_callback(self, victim):
        self.text = victim.id
		

if __name__ == "__main__":
    ReportButton()
