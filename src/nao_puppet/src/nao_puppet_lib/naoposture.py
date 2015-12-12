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
from std_msgs.msg import String

class _Constants:
    msg_topic = "posture"


class NaoPosture:
    def __init__(self, ip, port):
        self.__proxy = ALProxy("ALRobotPosture", ip, port)
        self.__subs = rospy.Subscriber(
            _Constants.msg_topic,
            String,
            self.go_to_posture)
        
        #Print all available postures
        rospy.loginfo(
            "Available postures: {} ".format(self.__proxy.getPostureList()) )


    def go_to_posture(self, msg):
        self.__proxy.goToPosture(msg.data, 1.0)