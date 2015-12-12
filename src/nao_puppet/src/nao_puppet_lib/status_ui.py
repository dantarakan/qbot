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

def _bool_to_check_state(val):
    if val:
        return QtCore.Qt.Checked
    else:
        return QtCore.Qt.Unchecked


class StatusUI( QtGui.QGroupBox ):
    
    stop_behaviours_signal = QtCore.pyqtSignal()

    def __init__(self):
        super(StatusUI, self).__init__("Status")

        #Needed widgets here
        self.motors_checkbox = QtGui.QCheckBox("Motors")
        self.breathing_checkbox = QtGui.QCheckBox("Breathing")
        self.volume_slider = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.volume_slider.setRange(0, 100)
        self.posture_label = QtGui.QLabel()
        self.behaviours_label = QtGui.QLabel()
        self.stop_button = QtGui.QPushButton("Stop Behaviours")
        
        self.posture_label.setMinimumWidth(120)
        self.posture_label.setMaximumWidth(120)
        self.behaviours_label.setMinimumWidth(120)
        self.behaviours_label.setMaximumWidth(120)
        
        #Show checkbox signals as ours
        self.motors_signal = self.motors_checkbox.clicked
        self.breathing_signal = self.breathing_checkbox.clicked
        self.volume_signal = self.volume_slider.valueChanged
        self.stop_behaviours_signal = self.stop_button.clicked        
        
        #Layout 
        layout = QtGui.QVBoxLayout()
        hbox = QtGui.QHBoxLayout()
        hbox.addWidget(self.motors_checkbox)
        hbox.addWidget(self.breathing_checkbox)

        hbox.addWidget( QtGui.QLabel("  Volume:") )
        hbox.addWidget(self.volume_slider)
        layout.addLayout(hbox)
        
        hbox = QtGui.QHBoxLayout()
        hbox.addWidget(QtGui.QLabel("Posture: ") )
        hbox.addWidget(self.posture_label )
        hbox.addWidget(QtGui.QLabel("Behaviours: ") )
        hbox.addWidget(self.behaviours_label )
        layout.addLayout(hbox)
        
        hbox = QtGui.QHBoxLayout()
        hbox.addWidget(self.stop_button)
        hbox.addStretch(0)
        layout.addLayout(hbox)


        self.setLayout(layout)

    @QtCore.pyqtSlot(bool, bool, int, str, list)
    def update_status(self, motors, breathing, volume, posture, behaviours):
        motors_state = _bool_to_check_state(motors)
        breathing_state = _bool_to_check_state( breathing )        
        
        self.motors_checkbox.setCheckState( motors_state )
        self.breathing_checkbox.setCheckState( breathing_state )
        
        self.volume_slider.blockSignals(True)
        self.volume_slider.setValue(volume)
        self.volume_slider.blockSignals(False)
        
        self.posture_label.setText( posture )
        self.behaviours_label.setText( str(behaviours) )
                
        self.stop_button.setEnabled( bool(behaviours ) )