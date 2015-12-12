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
from PyQt4 import QtCore, QtGui
from std_msgs.msg import String, Bool, UInt8, Empty
from sensor_msgs.msg import Image
from nao_puppet.msg import MoveHead, Status

class _Constants:
    speech_topic = "speech"
    posture_topic = "posture"
    behaviour_topic = "behaviour"
    camera_topic = "camera"
    move_head_topic = "move_head"
    status_topic = "status"
    motors_topic = "set_motors"
    breathing_topic = "set_breathing"
    volume_topic = "set_volume"
    stop_behaviours_topic = "stop_behaviours"

        
class RospyRelay( QtCore.QObject ):
    camera_signal = QtCore.pyqtSignal(QtGui.QImage)
    status_signal = QtCore.pyqtSignal(bool, bool, int, str, list)

    def __init__(self):
        super(RospyRelay, self).__init__()

        self.speech_pub = rospy.Publisher(
            _Constants.speech_topic,
            String,
            latch=True )
        
        self.posture_pub = rospy.Publisher(
            _Constants.posture_topic,
            String,
            latch=True )
    
        self.behaviour_pub = rospy.Publisher(
            _Constants.behaviour_topic,
            String,
            latch=True )
        
        self.move_head_pub = rospy.Publisher(
            _Constants.move_head_topic,
            MoveHead,
            latch=True)
        
        self.motors_pub = rospy.Publisher(
            _Constants.motors_topic,
            Bool,
            latch=True)
            
        self.breathing_pub = rospy.Publisher(
            _Constants.breathing_topic,
            Bool,
            latch=True)
            
        self.volume_pub = rospy.Publisher(
            _Constants.volume_topic,
            UInt8,
            latch=True)
        
        self.stop_behaviours_pub = rospy.Publisher(
            _Constants.stop_behaviours_topic,
            Empty,
            latch=True)
        
        self.camera_sub = rospy.Subscriber(
            _Constants.camera_topic,
            Image,
            self.relay_image )
        
        self.status_sub = rospy.Subscriber(
            _Constants.status_topic,
            Status,
            self.relay_status )

    @QtCore.pyqtSlot(str)
    def relay_speech(self, msg):
        self.speech_pub.publish( str(msg) )
    
    @QtCore.pyqtSlot(str)
    def relay_posture(self, msg):
        self.posture_pub.publish( str(msg) )
    
    @QtCore.pyqtSlot(str)
    def relay_behaviour(self, msg):
        self.behaviour_pub.publish( str(msg) )
    
    @QtCore.pyqtSlot(float, float)
    def relay_head_move(self, pitch, yaw):
        msg = MoveHead()
        msg.pitch = pitch
        msg.yaw = yaw
        
        self.move_head_pub.publish(msg)
        
    @QtCore.pyqtSlot(bool)
    def relay_motors(self, value):       
        self.motors_pub.publish(value)
        
    @QtCore.pyqtSlot(bool)
    def relay_breathing(self, value):       
        self.breathing_pub.publish(value)
        
    @QtCore.pyqtSlot(int)
    def relay_volume(self, value):       
        self.volume_pub.publish(value)
    
    @QtCore.pyqtSlot()
    def relay_stop_behaviours(self):       
        self.stop_behaviours_pub.publish()
    
    def relay_image(self, ros_img):
        if ros_img.encoding == "rgb8":
            img_format = QtGui.QImage.Format_RGB888
        else:
            img_format = QtGui.QImage.Format_Invalid
        
        #Convert to Qt
        qt_img = QtGui.QImage(
            ros_img.data,
            ros_img.width,
            ros_img.height,
            img_format )
        
        self.camera_signal.emit(qt_img)
        
    def relay_status(self, status):
        self.status_signal.emit(
            status.motors,
            status.breathing,
            status.volume,
            status.posture,
            status.behaviours)