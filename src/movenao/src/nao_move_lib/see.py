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
import vision_definition
import threading

class _Constants:
    topic = "camera"
    kCameraSelectID = 18
    #camera parameter
    unique_name = "nao_teleop_camera"
    resolution = vision_definitions.kQQVGA
    colour_space = vision_definitions.kRGBColorSpace
    fps = 5
    
class NaoCamera:
    def __init__(self, ip, port):
    #naoqi stuff
        self.camera_proxy = ALProxy("ALVideoDevice", ip, port)
        
        #Thread stuff
        self.__running = True
        self.__thread = threading.Thread(target=self.__run)
        self.__thread.start()

	def __run(self):
    #Unsubscribe any previous instance of this process
    self.__proxy.unsubscribeAllInstances( _Constants.unique_name )

    #Subscribe and id for image
    name_id = self.__proxy.subscribe(
        _Constants.unique_name,
        _Constants.resolution,
        _Constants.colour_space,
        _Constants.fps )

    while self.__running:
        #Receive image data
        received = self.__proxy.getImageRemote( name_id )

        #Unpack image data
        width = received[0]
        height = received[1]
        layers = received[2]
        encoding = received[3]            
        raw_data = received[6]

        #Convert to ROS image
        img = Image()
        img.header.frame_id = "nao_head"
        img.header.stamp = rospy.Time.now()
        img.width = width
        img.height = height
        img.step = width * layers
        
        #print( "yaw fov: {} {}".format(received[8], received[10]) )
        #print( "pitch fov: {} {}".format(received[9], received[11]) )
        if encoding == vision_definitions.kYUVColorSpace:
            img.encoding = "mono8"
        elif encoding == vision_definitions.kRGBColorSpace:
            img.encoding = "rgb8"
        elif encoding == vision_definitions.kBGRColorSpace:
            img.encoding = "bgr8"
        elif encoding == vision_definitions.kYUV422ColorSpace:
            img.encoding = "yuv422"
        else:
            rospy.logerr(
                "Received unknown encoding: {}".format(encoding) )
        img.data = raw_data
        
        #Publish it
        self.__pub.publish(img)

    self.__proxy.unsubscribe( name_id )
    
    def stop(self):
    self.__running = False
    self.__thread.join()
        
		

