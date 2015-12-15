#!/bin/bash

rostopic pub -1 /CNC_2_SPC qbot/SpcCmd "What is your name?" 21
rostopic pub -1 /SPC_2_NLP qbot/SpcNLP "My name is Yiannis"
rostopic pub -1 /SPC_2_NLP qbot/SpcNLP "Nice to meet you, Yiannis!" 21
rostopic pub -1 /SPC_2_NLP qbot/SpcNLP "Nice to meet you too"
rostopic pub -1 /CNC_2_SPC qbot/SpcCmd "How are you?" 21
rostopic pub -1 /SPC_2_NLP qbot/SpcNLP "I'm OK"
rostopic pub -1 /CNC_2_SPC qbot/SpcCmd "How old are you?" 21
rostopic pub -1 /SPC_2_NLP qbot/SpcNLP "I am 40 years old"
rostopic pub -1 /CNC_2_SPC qbot/SpcCmd "How many hours did you sleep last night?" 21
rostopic pub -1 /SPC_2_NLP qbot/SpcNLP "I slept 6 hours"
rostopic pub -1 /CNC_2_SPC qbot/SpcCmd "What is the last thing you ate, and when?" 21
rostopic pub -1 /SPC_2_NLP qbot/SpcNLP "I ate a pizza at 5 PM"
rostopic pub -1 /CNC_2_SPC qbot/SpcCmd "What is the last thing you drunk, and when?" 21
rostopic pub -1 /SPC_2_NLP qbot/SpcNLP "I drunk a beer at 6 PM"
