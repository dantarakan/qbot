#!/bin/bash

rostopic list

rostopic pub -1 /NAV_2_CNC qbot/NavRes 0
rostopic pub -1 /NAV_2_CNC qbot/NavRes 1
rostopic pub -1 /SPC_2_CNC qbot/SpcRes "Hi I am patient" 0
rostopic pub -1 /SPC_2_CNC qbot/SpcRes "Nice to meet you too" 1
rostopic pub -1 /SPC_2_CNC qbot/SpcRes "I am fine thank you!" 1
rostopic pub -1 /SPC_2_CNC qbot/SpcRes "3 years old" 1
rostopic pub -1 /SPC_2_CNC qbot/SpcRes "25 hours" 1
rostopic pub -1 /SPC_2_CNC qbot/SpcRes "Awesome service" 1
