#!/bin/bash
rostopic pub -1 /facedetection movenao/Face_detection True
sleep 5
rostopic pub -1 /facedetection movenao/Face_detection False
sleep 5

