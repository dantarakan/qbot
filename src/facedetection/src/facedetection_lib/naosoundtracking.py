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
from naoqi import ALModule, ALProxy # microphone and speaker
from humantrack.msg import MoveHead,Face_detection
from std_msgs.msg import String
import alsaaudio # signal processing, a python module
import time
class _Constants:
    head_speed = 0.2 # Between 0-1
    yaw = 1.5
    pitch = 0.3
    yaw_limits = [-2.0857,  2.0857]
    pitch_limits = [-0.6720,  0.5149]
    #topic = "move_head"
    face_det_topic = "facedetection"
    face_res_topic = "faceresult"
    face_track_topic = "/CNC_2_FACETRACK"

class NaoSoundTrack(ALModule):
    '''
    This is the module that connects remotely to the NAO microphones
    '''
    def __init__(self, name, ip, port):
        #Init parent
        super(NaoSoundTrack, self).__init__(name)
        
        self.__ip = ip
        self.__port = port
        
        #self.__pub = rospy.Publisher(_Constants.topic, MoveHead , latch=True, queue_size =1000)
        
        self.__facepub = rospy.Publisher(_Constants.face_det_topic, Face_detection , latch=True, queue_size =1000)
        
        self.__CNCsub = rospy.Subscriber(_Constants.face_track_topic, String, self.start_sound_track)
        
        self.__faceRessub = rospy.Subscriber(_Constants.face_res_topic, Face_detection, self.stop_sound_track)
       
    
    def start_sound_track(self, msg):
        self.__proxyTTS = ALProxy("ALAnimatedSpeech", self.__ip, self.__port)
        
        # set the local configuration
        sayconfig = {"bodyLanguageMode":"contextual"}
        
        self.__proxyTTS.say("Can you help me find you by clapping your hand?", sayconfig)
        
        self.__proxyMotion = ALProxy("ALMotion", self.__ip, self.__port)
        
        #initialise microphone
        #self.__audioProxy = ALProxy("ALAudioDevice", self.__ip, self.__port)
        #initialise soundsourcelocalisation
        self.__sslProxy = ALProxy("ALSoundLocalization", self.__ip, self.__port)
        #initialise almemory
        self.__memoryProxy = ALProxy("ALMemory", self.__ip, self.__port)
        #debugging purpose
        #self.__audioProxy.setClientPreferences( self.getName() , 16000, 3, 0 )
        #self.__audioProxy.subscribe(self.getName())
        
        #configure sound detection
        self.__sslProxy.setParameter("Sensitivity",0.1)

        #callback from memory      
        try:
            self.__memoryProxy.unsubscribeToEvent("ALSoundLocalization/SoundLocated","soundtracking")
        except:
            pass
        
        self.__sslProxy.subscribe("sound_source_locator")
        self.__memoryProxy.subscribeToMicroEvent(
            "ALSoundLocalization/SoundLocated",
            self.getName(), 
            "AnotherUserDataToIdentifyEvent", 
            "sound_callback")

              
    def stop_sound_track(self, msg):
        self.__memoryProxy.unsubscribeToMicroEvent("ALSoundLocalization/SoundLocated",self.getName())
        self.__sslProxy.unsubscribe("sound_source_locator")
        rospy.logwarn("sound localisation stopped")
        #self.__audioProxy.unsubscribe(self.getName())
	
    def	reset_sensitivity(self,msg):
		if(msg.enable):
			self.__sslProxy.setParameter("Sensitivity", msg.sensitivity)
			rospy.logwarn(msg.sensitivity)
			
    def sound_callback(self, event, val, msg):
        """Deal with sound localisation event"""
        rospy.loginfo('sound location' + str(val[1][0])+str(val[1][1]))
        rospy.loginfo('confidence' + str(val[1][2]))    
        #self.__TTSproxy.say("sound detected")
        #self.__pub.publish(val[1][0],val[1][1])
        
        rospy.logwarn("Moving Head to Sound")
        
        joint_names = ["HeadYaw", "HeadPitch"]
        current = self.__proxyMotion.getAngles( joint_names, False )
        yaw = current[0] + val[1][0]
        pitch = current[1] + val[1][1]

        #Make sure we don't exceed angle limits
        if yaw < _Constants.yaw_limits[0]:
            yaw = _Constants.yaw_limits[0]
        elif yaw > _Constants.yaw_limits[1]:
            yaw = _Constants.yaw_limits[1]

        if pitch < _Constants.pitch_limits[0]:
            pitch = _Constants.pitch_limits[0]
        elif pitch > _Constants.pitch_limits[1]:
            pitch = _Constants.pitch_limits[1]

        self.__proxyMotion.setStiffnesses("Head", 1.0)
        self.__proxyMotion.angleInterpolationWithSpeed(
            joint_names,
            [ yaw, pitch ],
            _Constants.head_speed)
        time.sleep(2.0)
        #self.__proxyMotion.setStiffnesses("Head", 0.0)
        
        # Start face tracking
        self.__facepub.publish(True , 0.1)
        
        

