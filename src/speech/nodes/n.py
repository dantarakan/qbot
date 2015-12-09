#!/usr/bin/env python
#
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

import sys
import rospy
from naoqi import ALBroker
from nao_speech_lib import NaoMic

if __name__ == "__main__":
    rospy.init_node("NaoMic", sys.argv)

    #Get parameters
    ip = rospy.get_param("nao_ip", "192.168.1.125")
    #ip = rospy.get_param("nao_ip", "127.0.0.1")
    port = rospy.get_param("nao_port", 9559)
    #Start naoqi broker
    broker = ALBroker("NaoMicBroker", "0.0.0.0", 0, ip, port)

    rospy.loginfo("Starting NaoMic")    
    global mic
    mic = NaoMic("mic", ip, port)
    mic.start()
    rospy.loginfo("NaoMic started")
    
    rospy.spin()
    
    rospy.loginfo("NaoMic stopped")
    broker.shutdown()
