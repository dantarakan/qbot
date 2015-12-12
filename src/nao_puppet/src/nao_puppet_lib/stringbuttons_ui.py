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
    max_buttons_per_line = 4

class _StringButton(QtGui.QPushButton):
    def __init__(self, label, signal_text, signal):
        super(_StringButton, self).__init__(label)
        self.signal = signal
        self.signal_text = signal_text
        
        #Connect basic signal to reemit
        self.clicked.connect(self.reemit)
        
    @QtCore.pyqtSlot()
    def reemit(self):
        self.signal.emit( self.signal_text )
        
class StringButtons( QtGui.QGroupBox ):
    def __init__(self, label, entries, signal):
        super(StringButtons, self).__init__(label)
        
        if type(entries) == list:
            entries = { x:x for x in entries }
        
        #Create layout
        vbox = QtGui.QVBoxLayout()
        hbox = QtGui.QHBoxLayout()
        for label, signal_text in entries.items():
            button = _StringButton(label, signal_text, signal)
            hbox.addWidget(button)
            if hbox.count() >= _Constants.max_buttons_per_line:
                vbox.addLayout(hbox)
                hbox = QtGui.QHBoxLayout()
        
        if hbox.count() > 0:
            hbox.addStretch(0)
            vbox.addLayout(hbox)
        self.setLayout( vbox )
    