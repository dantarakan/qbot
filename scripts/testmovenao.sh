#!/bin/bash

rostopic pub -1 /joy movenao/Walk_control 0 0
rostopic pub -1 /posture std_msgs/String "Sit"
sleep 5
rostopic pub -1 /posture std_msgs/String "Stand"
sleep 5
rostopic pub -1 /joy movenao/Walk_control 10 0
sleep 2
rostopic pub -1 /joy movenao/Walk_control 0 0
rostopic pub -1 /joy movenao/Walk_control -- -10 0
sleep 2
rostopic pub -1 /joy movenao/Walk_control 0 0
rostopic pub -1 /posture std_msgs/String "Sit"
