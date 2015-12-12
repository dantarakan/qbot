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
from .stringbuttons_ui import StringButtons

class _Constants:
    fillers = [
        "Let me think",
        "Hang on a sec",
        "What was that?",
        "I beg your pardon?",
        "Okay...",
        "Interesting...",
        "I see...",
        "Right",
        "Ah",
        "Yes",
        "No",
        "I don't know",
        "Hello!",
        "Excellent",
        "Brilliant",
        "Why is that?"]
    
    small_talk = [
        "INTRODUCTION -----",
        "My name is Junior",
        "Nice to meet you",
        "How are you?",
        "How's your family?",
        "What's your name?",
        "How long have you been here?",
        "What is your age?",
        "Oops! I shouldn't ask that to a lady",
        "Would you like to know how old I am?",
        "I am 7 years old in robotic years",
        "Have you seen the weather today?",
        "Are you comfortable?",
        "How was your weekend?",
        "How was your night?",
        "Is there anything you want to talk about?",
        "What is your favourite film?",
        "My favourite film is Wallee",
        "What is your favourite song?",
        "Could you sing it for me?",
        "What's the best thing about being in the hospital?",
        "MEMORY [SHORT-TERM] -----",
        "My battery ran out and I've lost track of time",
        "Do you know what day is today?",
        "Let me think... Today must be...",
        "What time is it?",
        "What are your plans for today?",
        "What are you looking forward to today?",
        "Have you spoken to anyone else in the ward today?",
        "Have you seen your doctors today?",
        "What is the name of your doctor?",
        "What is the name of your nurse?",
        "Do you think time goes slowly?",
        "MEMORY [POETRY] -----",
        "I like poetry, do you remember any poem?",
        "Do you want to hear a few verses of my favourite poem?",
        "If you can fill the unforgiving minute. With sixty seconds worth of distance run. Yours is the Earth and everything that's in it. And, which is more, you'll be a Man, my son!",
        "When was your last holiday?",
        "MEMORY [OTHER LONG TERM] -----",
        "What is your favourite place?",
        "When did you spend your holidays as a child?",
        "JOKES -----",
        "Do you want to hear a joke?",
        "I know a really good joke",
        "Can you tell me a joke?",
        "Do you know any jokes?",
        "------",
        "How do you know that carrots are good for your eyesight?",
        "Have you ever seen a rabbit with glasses?",
        "------",
        "What do snowmen have for breakfast?",
        "Snowflakes",
        "------",
        "What do you have in December that you don't have in any other month?",
        "The letter D",
        "------",
        "What does one ho plus two ho make?",
        "A jolly Santa",
        "------",
        "Three elderly ladies were at the doctor for a cognitive reasoning test.",
        "The doctor says to the first gal, \"What is three times three?\"",
        "\"297,\" was her prompt reply. \"Ummm humm,\" says the doc.",
        "The doctor says to the second lady, \"It's your turn now. What is three times three?\"",
        "\"Friday,\" replies the second lady. \"Ummm humm...\"",
        "Then the doc says to the third, \"Okay, mam, your turn. What's three times three?\"",
        "\"Nine,\" she says. \"That's wonderful!\" says the doc. \"Tell me, how did you get that?\"",
        "\"Simple,\" she says, beaming... \"I subtracted 297 from Friday!\"",
        "------",
        "An elderly couple was driving across the country. While the woman was behind the wheel, the couple was pulled over by the highway patrol.",
        "\"Ma'am, did you know you were speeding?\" the officer said.",
        "The woman, hard of hearing, turned to her husband and asked, \"What did he say?\"",
        "\"He said you were speeding!\" the old man yelled.",
        "The patrolman then asked, \"May I see your license?\"",
        "The woman turned to her husband again, \"What did he say?\"",
        "The old man yelled back, \"He wants to see your license!\"",
        "The woman then gave the officer her license.",
        "\"I see you are from Arkansas,\" the patrolman said. \"I spent some time there once and went on a blind date with the ugliest woman I've ever seen.\"",
        "The woman turned to her husband again and asked, \"What did he say?\"",
        "The old man replied, \"He said he knows you!\"",
        "------",
        "A Horse goes into a bar and the bartender says \"Hey buddy, Why the Long Face?\"",
        "------",
        "Where do you find a one legged dog?",
        "Where you left it.",
        "------",
        "What is pink and fluffy?",
        "Pink fluff",
        "And, what is blue and fluffy?",
        "Pink fluff holding its breath",
        "------",
        "Two muffins are in the oven. One says to the other \"God it's hot in here\"",
        "The other one replies \"Oh no... It's a talking muffin\"",
        "NEWS -----",
        "Would you like me to tell you about today's news?",
        "I've checked the news earlier",
        "LUNCHTIME -----",
        "What's the food like?",
        "That looks nice",
        "Have you tried it?",
        "Shall we try a bit?",
        "How does it taste?",
        "Is it too hot?",
        "It will make you strong",
        "Take another bite!",
        "Have you finished your food?",
        "It's a pity robots cannot eat food",
        "Would you like some music?",
        "EXERCISE -----",
        "Let's do some exercise!",
        "Do you want to do some exercise?",
        "First let's start by stretching",
        "Take 5 deep breaths",
        "Should we try the left arm?",
        "Should we try the right arm?",
        "Let me show you first and then your repeat",
        "Now is your turn",
        "You can relax your arms now",
        "And again...",
        "Good job!",
        "Go on...",
        "Are you tired? I'm exhausted",
        "DANCING -----",
        "Do you want to see me dance?",
        "I love dancing!",
        "Did you like it?",
        "DEPARTING -----",
        "It was lovely meeting you",
        "Take care",
        "I'm going to shut down now",
        "See you at lunchtime",
        "See you later this evening!" ]

        
class _InputRow(QtGui.QGroupBox):   
    def __init__(self, label, button_text, signal):
        super(_InputRow, self).__init__(label)
        #Create the two widgets that require this signal
        self.edit_widget = QtGui.QLineEdit()
        self.button = QtGui.QPushButton(button_text)
        self.signal = signal
        
        #Set the layout for this
        layout = QtGui.QHBoxLayout()
        layout.addWidget(self.edit_widget)
        layout.addWidget(self.button)
        self.setLayout(layout)      
        
        #Connect basic signal to reemit
        self.edit_widget.returnPressed.connect(self.reemit)
        self.edit_widget.textEdited.connect(self.done_editing)
        self.button.clicked.connect(self.reemit)
        
        #Update button
        self.done_editing( )
    
    
    @QtCore.pyqtSlot(str)
    def done_editing(self, text = ""):
        empty = not self.edit_widget.text()
        self.button.setDisabled(empty)
        
            
    @QtCore.pyqtSlot()
    def reemit(self):
        self.signal.emit( self.edit_widget.text() )
        self.edit_widget.clear()
        self.done_editing()

class _SmallTalk(QtGui.QGroupBox):
    def __init__(self, label,  button_text, signal):
        super(_SmallTalk, self).__init__( label )
        
        self.signal = signal
        self.list_view = QtGui.QListWidget()
        self.button = QtGui.QPushButton( button_text )
        
        #Populate List
        self.populate_list()
        
        #Make sure we cannot scroll right
        self.list_view.setHorizontalScrollBarPolicy(
            QtCore.Qt.ScrollBarAlwaysOff )
        
        #Set layout
        layout = QtGui.QVBoxLayout()
        layout.addWidget( self.list_view ) 
        button_row = QtGui.QHBoxLayout()
        button_row.addStretch(0)
        button_row.addWidget(self.button)
        layout.addLayout( button_row )
        
        self.setLayout(layout)
        
        #Connect signals
        self.list_view.activated.connect(self.reemit)
        self.list_view.clicked.connect( self.selected )
        self.button.clicked.connect(self.reemit)
        
        #Update button
        self.selected()
    
    def populate_list(self):
        for text in _Constants.small_talk:
            item = QtGui.QListWidgetItem(text)

            if text.endswith("---"):
                item.setFlags(QtCore.Qt.NoItemFlags)
            
            self.list_view.addItem(item)
            
    @QtCore.pyqtSlot(QtCore.QModelIndex)
    def selected(self, index=None):
        empty = self.list_view.currentItem() is None
        self.button.setDisabled(empty)
            
    @QtCore.pyqtSlot()
    def reemit(self):
        self.signal.emit( self.list_view.currentItem().text() )
        self.list_view.setCurrentItem(None)
        self.selected()
        
class SpeechUI( QtGui.QWidget ):
    speech_signal = QtCore.pyqtSignal(str)

    def __init__(self):
        super(SpeechUI, self).__init__()

        #Init UI here
        layout = QtGui.QVBoxLayout()
        layout.addWidget(  self.get_fillers_row() )
        layout.addWidget(  self.get_input_row() )
        layout.addWidget(  self.get_small_talk() )
        
        self.setLayout(layout)

    def get_fillers_row(self):
        return StringButtons(
            "Filler speech",
            _Constants.fillers,
            self.speech_signal)
    
    def get_input_row(self):
        return _InputRow("General speech", "Say it!", self.speech_signal)

    
    def get_small_talk(self):
        return _SmallTalk("Small talk", "Say it!", self.speech_signal)
