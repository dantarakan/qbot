#!/usr/bin/env python

import sys
import rospy
from nlp_lib import NLP

if __name__ == "__main__":
    rospy.init_node("NLP")

    njm = NLP()
    rospy.logwarn("NLP started")
    rospy.spin()
    rospy.logwarn("NLP stopped")
