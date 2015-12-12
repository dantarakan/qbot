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
import threading
from naoqi import ALProxy
from std_msgs.msg import Bool, Empty, UInt8
from nao_puppet.msg import Status

class _Constants:
    status_topic = "status"
    motors_topic = "set_motors"
    breathing_topic = "set_breathing"
    volume_topic = "set_volume"
    stop_behaviours_topic = "stop_behaviours"
    rate = 20 #Hz, status updates per second


class NaoStatus:

    def __init__(self, ip, port):
        #naoqi stuff
        self.__motion = ALProxy( "ALMotion", ip, port )
        self.__audio = ALProxy( "ALAudioDevice", ip, port )
        self.__posture = ALProxy( "ALRobotPosture", ip, port )
        self.__behaviours = ALProxy("ALBehaviorManager", ip, port)

        #ROS stuff
        self.__rate = rospy.Rate(_Constants.rate)
        self.__pub = rospy.Publisher(
            _Constants.status_topic,
            Status,
            latch=True)
        
        self.__motors_sub = rospy.Subscriber(
            _Constants.motors_topic,
            Bool,
            self.__set_motors)
    
        self.__breathing_sub = rospy.Subscriber(
            _Constants.breathing_topic,
            Bool,
            self.__set_breathing)
    
        self.__volume_sub = rospy.Subscriber(
            _Constants.volume_topic,
            UInt8,
            self.__set_volume)
            
        self.__stop_behaviours_sub = rospy.Subscriber(
            _Constants.stop_behaviours_topic,
            Empty,
            self.__stop_behaviours)

        #Thread stuff
        self.__running = True
        self.__thread = threading.Thread(target=self.__run)
        self.__thread.start()

    def __run(self):
        while self.__running:
            stiffnesses =  self.__motion.getStiffnesses("Body")
            
            msg = Status()
            msg.motors = not all(x==0 for x in stiffnesses)
            msg.breathing = self.__motion.getBreathEnabled("Arms")
            msg.volume = self.__audio.getOutputVolume()
            msg.posture = self.__posture.getPosture()
            msg.behaviours = self.__behaviours.getRunningBehaviors()
            
            self.__pub.publish(msg)
            
            self.__rate.sleep()
        
    def __set_motors(self, msg):
        value = 1.0 if msg.data else 0.0
        self.__motion.setStiffnesses("Body", value)

    def __set_breathing(self, msg):
        self.__motion.setBreathEnabled( "Arms", msg.data )
        #self.__motion.setBreathEnabled( "Head", msg.data )
    
    def __set_volume(self, msg):
        value = msg.data
        if value > 100:
            value = 100
        self.__audio.setOutputVolume(value)
        
    def __stop_behaviours(self, msg):
        self.__behaviours.stopAllBehaviors()

    def stop(self):
        self.__running = False
        self.__thread.join()
