#!/bin/bash

rostopic pub -1 /CNC_2_SPC nlp/CncStatus 20
rostopic pub -1 /CNC_2_SPC nlp/SpcCmd "What is your name?" 20
rostopic pub -1 /SPC_2_NLP nlp/SpcNLP "My name is Yiannis"
rostopic pub -1 /CNC_2_SPC nlp/CncStatus 21
rostopic pub -1 /SPC_2_NLP nlp/SpcNLP "Nice to meet you, Yiannis, are you ready to start?" 21
rostopic pub -1 /SPC_2_NLP nlp/SpcNLP "Yes"
rostopic pub -1 /CNC_2_SPC nlp/CncStatus 22
rostopic pub -1 /CNC_2_SPC nlp/SpcCmd "How old are you?" 22
rostopic pub -1 /SPC_2_NLP nlp/SpcNLP "I am 40 years old"
rostopic pub -1 /CNC_2_SPC nlp/SpcCmd "How many hours did you sleep last night?" 22
rostopic pub -1 /SPC_2_NLP nlp/SpcNLP "I slept 6 hours"
rostopic pub -1 /CNC_2_SPC nlp/SpcCmd "What is the last thing you ate, and when?" 22
rostopic pub -1 /SPC_2_NLP nlp/SpcNLP "I ate a pizza at 5 PM"
rostopic pub -1 /CNC_2_SPC nlp/SpcCmd "What is the last thing you drunk, and when?" 22
rostopic pub -1 /SPC_2_NLP nlp/SpcNLP "I drunk a beer at 6 PM"
