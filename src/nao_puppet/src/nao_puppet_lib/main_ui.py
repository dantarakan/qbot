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

import rospkg
from PyQt4 import QtGui
from .speech_ui import SpeechUI
from .activity_ui import ActivityUI
from .camera_ui import CameraUI
from .status_ui import StatusUI
from .rospy_relay import RospyRelay


def _get_icon_path():
     rospack = rospkg.RosPack()
     path = rospack.get_path("nao_puppet")
     return path + "/share/icon.png"

class PuppetUI( QtGui.QWidget ):
    
    def __init__(self):
        super(PuppetUI, self).__init__()

        
        #Start rospy relayer
        self.__relay = RospyRelay()
        
        #Init widgets
        self.__speech = SpeechUI()
        self.__activity = ActivityUI()
        self.__camera = CameraUI()
        self.__status = StatusUI()

        #Connect everything
        self.__connect_signals()
        
        #Set layout
        self.__set_layout()
        
        self.setWindowTitle('NaoPuppet')
        self.setWindowIcon( QtGui.QIcon(_get_icon_path() ) )

        self.show()

    def __connect_signals(self):
        self.__speech.speech_signal.connect( self.__relay.relay_speech )
        self.__activity.posture_signal.connect( self.__relay.relay_posture )
        self.__activity.behaviour_signal.connect( self.__relay.relay_behaviour )
        self.__camera.move_head_signal.connect( self.__relay.relay_head_move)
        self.__status.motors_signal.connect( self.__relay.relay_motors )
        self.__status.breathing_signal.connect( self.__relay.relay_breathing )
        self.__status.volume_signal.connect( self.__relay.relay_volume )
        self.__status.stop_behaviours_signal.connect(
            self.__relay.relay_stop_behaviours )
        
        self.__relay.camera_signal.connect( self.__camera.receive_camera_image )
        self.__relay.status_signal.connect( self.__status.update_status)
        
        
    def __set_layout(self):
        #Prepare layout
        vbox = QtGui.QVBoxLayout()
        vbox.addWidget( self.__camera )
        vbox.addWidget( self.__activity )
        vbox.addWidget( self.__status )

        hbox = QtGui.QHBoxLayout()
        hbox.addWidget(self.__speech)
        hbox.addLayout(vbox)
        self.setLayout(hbox)
