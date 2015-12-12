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

from PyQt4 import QtCore, QtGui

class _Constants:
    yaw_fov = 0.531452775002
    pitch_fov = 0.415388375521

class _ClickLabel(QtGui.QLabel):
    def __init__(self, signal):
        super(_ClickLabel, self).__init__()
        self.signal = signal
        
    def setPixmap(self, pixmap):
        scaled = pixmap.scaled(self.size(),  QtCore.Qt.KeepAspectRatio )
        super(_ClickLabel, self).setPixmap(scaled)
    
    def resizeEvent(self, event):
        super(_ClickLabel, self).resizeEvent(event)
        pixmap = self.pixmap()
        if pixmap is not None:
            self.setPixmap(pixmap)
    
    def mousePressEvent(self, event):
        super(_ClickLabel, self).mousePressEvent(event)
        #Get pixmap
        pixmap = self.pixmap()
        if pixmap is None:
            return
            
        #Get centered coordinates
        x = event.x() - self.width()/2.0
        y = event.y() - self.height()/2.0
        
        #Normalise with respect to size of pixmap
        norm_x =  x / (pixmap.width()/2.0)
        norm_y = y / (pixmap.height()/2.0)
        
        #If the numbers are >1 then clicked outside of picture
        if abs(norm_x) > 1.0 or abs(norm_y) > 1.0:
            return
        
        # Convert to pitch and yaw by multiplying by fov
        pitch = norm_y * _Constants.pitch_fov
        yaw = -(norm_x * _Constants.yaw_fov)
        
        self.signal.emit(pitch, yaw)

class CameraUI( QtGui.QGroupBox ):

    move_head_signal = QtCore.pyqtSignal(float, float)
    
    def __init__(self):
        super(CameraUI, self).__init__("Camera")
        #Prepare camera view
        self.__camera_view = _ClickLabel(self.move_head_signal)
        self.__camera_view.setMinimumSize(320, 240)
        self.__camera_view.setMaximumSize(640, 480)  
    
        #Set layoyt
        layout = QtGui.QHBoxLayout()
        layout.addStretch(0)
        layout.addWidget( self.__camera_view )
        layout.addStretch(0)
        self.setLayout( layout )


    @QtCore.pyqtSlot(QtGui.QImage)
    def receive_camera_image(self, qt_img):
        pixmap = QtGui.QPixmap()
        pixmap.convertFromImage( qt_img )
        self.__camera_view.setPixmap(pixmap) 
    
