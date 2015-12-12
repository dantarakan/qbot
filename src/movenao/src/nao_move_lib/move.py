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
from movenao.msg import Walk_control
from std_msgs.msg import String

class _Constants:
    joy_topic = "joy"
    msg_topic = "posture"
    linear_factor = 0.7
    angular_factor = 0.5

class MoveNao:
    def __init__(self, ip, port):
        self.__proxy = ALProxy("ALMotion", ip, port)
        self.__proxyPosture = ALProxy("ALRobotPosture", ip, port)
        self.__walk_sub = rospy.Subscriber(
            _Constants.joy_topic,
            Walk_control,
            self.walk)
            
        self.__subs = rospy.Subscriber(
        _Constants.msg_topic,
        String,
        self.go_to_posture)
            
    def walk(self, msg):
        angular = msg.angular
        linear = msg.linear
       
        self.__proxy.move(
            linear * _Constants.linear_factor, # Forwards
            0.0, #Sideways
            angular * _Constants.angular_factor ) #Turning
            
    def go_to_posture(self, msg):
        self.__proxyPosture.goToPosture(msg.data, 1.0)

