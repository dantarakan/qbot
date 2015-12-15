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
from humantrack.msg import Face_detection
from std_msgs.msg import String


class _Constants:
    face_det_topic = "facedetection"
    face_res_topic = "faceresult"
    unique_name = "nao_camera"
    resolution = vision_definitions.kQVGA
    colour_space = vision_definitions.kRGBColorSpace
    fps = 5

class NaoFaceTrack:
    def __init__(self, ip, port):
        self.__proxyTTS = ALProxy("ALTextToSpeech", ip, port) #test only, will need to use publisher later
        self.__proxyVideo = ALProxy("ALVideoDevice", ip, port)
        self.__proxyFace = ALProxy("ALFaceDetection", ip, port)
        self.__proxyMemory = ALProxy("ALMemory", ip, port)		              
    	self.__face_sub = rospy.Subscriber( 
    		_Constants.face_det_topic, 
    		Face_detection, 
    		self.detect_face)		
        
        self.__facepub = rospy.Publisher(_Constants.face_res_topic, Face_detection , latch=True, queue_size =1000)
        	
        
    def detect_face(self, msg):
    	Foundface = False
    	if(msg.enable):
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
            speek = 0
            for i in range(0, 20):
                time.sleep(0.5)
                val = self.__proxyMemory.getData(memValue)

                #rospy.logwarn("***************")

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
                            if(speek == 0):
                                #self.__proxyTTS.say("Hello Human")
                                #self.__facepub.publish(True)
                                speek = 1
                    except Exception, e:
                        rospy.loginfo("faces detected, but it seems getData is invalid. ALValue ="+ val)
                        rospy.loginfo("Error msg %s", (str(e)))
                    break
                else:
                    if(speek == 1):
                        rospy.loginfo("No face detected")
                        #self.__proxyTTS.say("No face detected")
                        speek = 0
            self.__proxyFace.unsubscribe("TEST")	
            self.__proxyVideo.unsubscribe( name_id )
        else:
            rospy.loginfo("face detection disabled")
