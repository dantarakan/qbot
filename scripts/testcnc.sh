#!/bin/bash

rostopic list

rostopic pub -1 /GUI_2_CNC qbot/GuiRes 101
rostopic pub -1 /GUI_2_CNC qbot/GuiRes 1

rostopic pub -1 /NAV_2_CNC qbot/NavRes 0
rostopic pub -1 /NAV_2_CNC qbot/NavRes 1

rostopic pub -1 /GUI_2_CNC qbot/GuiRes 200

#rostopic pub -1 /NLP_2_CNC qbot/NLPRes "Hi I am patient zero. Nice to meet you too" 0
#rostopic pub -1 /NLP_2_CNC qbot/NLPRes "Woooloolololoooo!" 1
#rostopic pub -1 /NLP_2_CNC qbot/NLPRes "Woolalalalalllaaaaaaa!" 1
#rostopic pub -1 /NLP_2_CNC qbot/NLPRes "I am fine thank you!" 0
#rostopic pub -1 /NLP_2_CNC qbot/NLPRes "3 years old" 0
#rostopic pub -1 /NLP_2_CNC qbot/NLPRes "25 hours" 0
#rostopic pub -1 /NLP_2_CNC qbot/NLPRes "Awesome service" 0

