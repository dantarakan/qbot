#!/bin/bash

rostopic pub -1 /CNC_2_SPC qbot/SpcCmd "How old are you?" 21
rostopic pub -1 /SPC_2_NLP qbot/SpcNLP "I am six years old."
rostopic pub -1 /SPC_2_NLP qbot/SpcNLP "I want a break"
rostopic pub -1 /SPC_2_NLP qbot/SpcNLP "Please call the nurse!"
