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
import time
from naoqi import ALProxy
import vision_definitions
from movenao.msg import Walk_control, Face_detection
from std_msgs.msg import String


class _Constants:
    joy_topic = "joy"
    msg_topic = "posture"
    face_det_topic = "facedetection"
    linear_factor = 0.7
    angular_factor = 0.5
    unique_name = "nao_camera"
    resolution = vision_definitions.kQVGA
    colour_space = vision_definitions.kRGBColorSpace
    fps = 5

class MoveNao:
    def __init__(self, ip, port):
        self.__proxy = ALProxy("ALMotion", ip, port)
        self.__proxyPosture = ALProxy("ALRobotPosture", ip, port)
        self.__proxyTTS = ALProxy("ALTextToSpeech", ip, port) #test only, will need to use publisher later
        self.__proxyVideo = ALProxy("ALVideoDevice", ip, port)
        self.__proxyFace = ALProxy("ALFaceDetection", ip, port)
        self.__proxyMemory = ALProxy("ALMemory", ip, port)
        self.__walk_sub = rospy.Subscriber(
            _Constants.joy_topic,
            Walk_control,
            self.walk)
            
        self.__subs = rospy.Subscriber(
		    _Constants.msg_topic,
		    String,
		    self.go_to_posture)
		      

                
    	self.__face_sub = rospy.Subscriber( 
    		_Constants.face_det_topic, 
    		Face_detection, 
    		self.detect_face)		
            
    def walk(self, msg):
        angular = msg.angular
        linear = msg.linear
       
        self.__proxy.move(
            linear * _Constants.linear_factor, # Forwards
            0.0, #Sideways
            angular * _Constants.angular_factor ) #Turning
            
    def go_to_posture(self, msg):
        self.__proxyPosture.goToPosture(msg.data, 1.0)
        
    def detect_face(self, msg):
    	Foundface = False
    	if(msg.enable):
            rospy.loginfo("code executed here")
            #registering VIM
            #not sure this part is needed or handled by ALFaceDetection
            name_id = self.__proxyVideo.subscribe(
                _Constants.unique_name,
                _Constants.resolution,
                _Constants.colour_space,
                _Constants.fps )
            #subscribe to ALFaceDetection proxy GVM
            Period = 500
            self.__proxyFace.subscribe("TEST", Period, 0.0)
            #getting video buffer
            #getting detected result from ALMemory
            memValue = "FaceDetected"
            
            for i in range(0, 20):
                time.sleep(0.5)
                val = self.__proxyMemory.getData(memValue)

                rospy.loginfo("***************")

                # Check whether we got a valid output.
                if(val and isinstance(val, list) and len(val) >= 2):
                    
                    # We detected faces !
                    # For each face, we can read its shape info and ID.

                    # First Field = TimeStamp.
                    timeStamp = val[0]

                    # Second Field = array of face_Info's.
                    faceInfoArray = val[1]

                    try:
                        # Browse the faceInfoArray to get info on each detected face.
                        for j in range( len(faceInfoArray)-1 ):
                            faceInfo = faceInfoArray[j]

                            # First Field = Shape info.
                            faceShapeInfo = faceInfo[0]

                            # Second Field = Extra info (empty for now).
                            faceExtraInfo = faceInfo[1]
                            rospy.loginfo("  alpha %.3f - beta %.3f", faceShapeInfo[1], faceShapeInfo[2])  
                            rospy.loginfo("  alpha %.3f - beta %.3f", faceShapeInfo[3], faceShapeInfo[4])  
                            if(Foundface):
                                self.__proxyTTS.say("I can still see you")
                            else:
                                self.__proxyTTS.say("I can see you")
                                Foundface = True
                    except Exception, e:
                        rospy.loginfo("faces detected, but it seems getData is invalid. ALValue ="+ val)
                        rospy.loginfo("Error msg %s", (str(e)))
                else:
                    rospy.loginfo("No face detected")
                    self.__proxyTTS.say("Adam I want to hurt you")
                    Foundface = False
            self.__proxyFace.unsubscribe("TEST")	
            self.__proxyVideo.unsubscribe( name_id )
        else:
            rospy.loginfo("face detection disabled")
