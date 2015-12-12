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
from stringbuttons_ui import StringButtons

class _Constants:
    postures = {
        "Stand" : "Stand",
        "Walk"  : "StandInit",
        "Crouch": "Crouch" }
    
    arm_exercises = {
        "Stretch arms": "puppet_exercise/stretch_arms",
        "Left arm"    : "puppet_exercise/exercise_left_arm", 
        "Right arm"   : "puppet_exercise/exercise_right_arm" }

    dances = {
        "Taichi" : "taichi/dance",
        "ALIZ-E" : "alize-dance/dance",
        "Carol" : "classical_music/carol_of_the_bells",
        "Mozart" : "classical_music/mozart",
        "Beethoven" : "classical_music/beethoven",
        "Brahms" : "classical_music/brahms",
        "Silent Night": "classical_music/extra_carol1",
        "Bleak Mid-winter" : "classical_music/extra_carol2",
        "David's city" : "classical_music/extra_carol3" }

class ActivityUI( QtGui.QWidget ):
    posture_signal = QtCore.pyqtSignal(str)
    behaviour_signal = QtCore.pyqtSignal(str)

    def __init__(self):
        super(ActivityUI, self).__init__()
        
        #Instantiate sets of buttons
        posture_buttons = StringButtons(
            "Postures",
            _Constants.postures,
            self.posture_signal )
        
        arm_exercises_buttons = StringButtons(
            "Arm exercises",
            _Constants.arm_exercises,
            self.behaviour_signal)
        
        dances_buttons = StringButtons(
            "Dances and Music",
            _Constants.dances,
            self.behaviour_signal)
        
        #Set layout
        vbox = QtGui.QVBoxLayout()

        vbox.addWidget( posture_buttons )
        vbox.addWidget( arm_exercises_buttons )
        vbox.addWidget( dances_buttons)
        
        self.setLayout( vbox )
    
