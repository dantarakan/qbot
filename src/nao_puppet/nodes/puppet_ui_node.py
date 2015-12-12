#!/usr/bin/env python
#
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

import sys
import rospy
import signal
from PyQt4 import QtCore, QtGui
from nao_puppet_lib import PuppetUI

    
def sigint_handler( signal, frame ):
    rospy.signal_shutdown("SIGINT received")
    QtGui.QApplication.quit()


if __name__ == '__main__':
    app = QtGui.QApplication( sys.argv )
    rospy.init_node( "PuppetUI", sys.argv )
    
    
    # Sets up signal handling so SIGINT closes the application,
    # following the solution given at [1].  Sets up a custom signal
    # handler, and ensures that the Python interpreter runs
    # occasionally so the signal is handled.  The email thread at [2]
    # explains why this is necessary.
    #
    # [1] http://stackoverflow.com/questions/4938723/#4939113
    # [2] http://www.mail-archive.com/pyqt@riverbankcomputing.com/msg13757.html
    signal.signal( signal.SIGINT, sigint_handler )
    timer = QtCore.QTimer()
    # Forces the interpreter to run every 250ms
    timer.start( 250 )
    timer.timeout.connect( lambda: None )
    signal.signal( signal.SIGINT, sigint_handler )

    rospy.loginfo("Starting PuppetUI....")
    ui = PuppetUI()

    rospy.loginfo("PuppetUI. Press Ctrl+C to stop.")

    app.exec_()
    
    rospy.init_node("PuppetUI", sys.argv)
