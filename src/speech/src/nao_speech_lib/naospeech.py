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
from speech.msg import Speech_String # ros function, import 'Speech_String' type
from naoqi import ALModule, ALProxy # microphone and speaker
import alsaaudio # signal processing, a python module



class NaoMic(ALModule):
    '''
    This is the module that connects remotely to the NAO microphones
    '''
    def __init__(self, name, ip, port):
        #Init parent
        super(NaoMic, self).__init__(name)

        self.__ip = ip
        self.__port = port
        
        self.__pcm = alsaaudio.pcm = alsaaudio.PCM(
            alsaaudio.PCM_PLAYBACK,
            alsaaudio.PCM_NONBLOCK)
            
        self.__pcm.setrate( 16000 )
        self.__pcm.setchannels( 1 )
        self.__pcm.setformat( alsaaudio.PCM_FORMAT_S16_LE )
        self.__pcm.setperiodsize(1365)
    
    def __del__(self):
        self.stop()
    
    def __get_proxy(self):
        return ALProxy("ALAudioDevice", self.__ip, self.__port)

    def start(self):
        proxy = self.__get_proxy()
        proxy.setClientPreferences( self.getName() , 16000, 3, 0 )
        
        #This will call processRemote with new data from NAO
        proxy.subscribe(self.getName())

    def stop(self):
        #Unsubscribe from microphone
        proxy = self.__get_proxy()
        proxy.unsubscribe( self.getName() )

    
    def processRemote(self, channels, samples_by_channel, timestamp, raw_data ):

        self.__pcm.write(raw_data)
        


class NaoSpeech:
    def __init__(self, ip, port):
        rospy.loginfo('test: hello')
        self.__proxy = ALProxy("ALTextToSpeech", ip, port)
        self.__subs = rospy.Subscriber("/CNC_2_SPC", SpcCmd, self.say)


    def say(self, msg):
    	rospy.loginfo(msg.question+ '0')
        self.__proxy.say( msg.question)
        #rospy.loginfo(msg.Speech_String+ '1')
        #self.__proxy.sayToFile("This is another sample text", "/tmp/sample_text.wav")
        #rospy.loginfo(msg.Speech_String+ '2')
    def sayString(self, msg):
    	rospy.loginfo(msg+ ' start')
        self.__proxy.say( msg)
        rospy.loginfo(msg+ ' end')
