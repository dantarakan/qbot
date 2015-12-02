#!/usr/bin/env python

# Copyright (c) 2014 Miguel Sarabia
# Imperial College London
#
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
#

import rospy
from naoqi import ALProxy
from sensor_msgs.msg import Joy

class _Constants:
    joy_topic = "joy"

    linear_factor = 0.7
    angular_factor = 0.5

class Move:
    def __init__(self, ip, port):

        self.__proxy = ALProxy("ALMotion", ip, port)

        self.__walk_sub = rospy.Subscriber(
            _Constants.joy_topic,
            Joy,
            self.walk)


    def walk(self, msg):
        angular = msg.axes[0]
        linear = msg.axes[1]
       
        self.__proxy.move(
            linear * _Constants.linear_factor, # Forwards
            0.0, #Sideways
            angular * _Constants.angular_factor ) #Turning

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('move')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = Move()
    except rospy.ROSInterruptException: pass

