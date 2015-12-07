# Q-Bot

Q-Bot is self-navigating humanoid robot to be used for conducting questionnaires in hospitals and research projects.

Here lies the ROS project, built by Sze Tan, Hanyi Hu, Daniil Tarakanov, Alxandre Benoit and Guang Yang, as a part of a Human-Centred Robotics course at Imperial College London.

#### Useful directories:
##### Nao puppet:
 
```sh
~/opt/extra_ws/src/nao_puppet
```

##### Q-Bot:

```sh
~/catkin_ws/src/qbot
```

##### NLP:

```sh
~/catkin_ws/src/nlp
```

#### Build:
```sh
cd ~/catkin_ws
catkin_make
```

#### Launch Q-Bot:
```sh
source ~/catkin_ws/devel/setup.bash
roslaunch ~/catkin_ws/src/qbot/launch/qbot.launch

OR

roslaunch qbot qbot.launch
```
This can be also be used to launch any other package. Simply replace *qbot* by the package name.

#### Launch Nao Puppet:
```sh
source /opt/extra_ws/devel/setup.bash
roslaunch /opt/extra_ws/src/nao_puppet/launch/nao_puppet.launch
```

#### Launch MoveNao:
Launch Choregraphe
```sh
choregraphe
```
Note the port number from the terminal.
In moveNao/nodes/naomotion_node.py change port number on line 37 to the new number.
Also change the port number in ~/catkin_ws/src/movenao/launch/movenao.launch
In a new terminal:
```sh
catkin_make
source ~/catkin_ws/devel/setup.bash
roslaunch ~/catkin_ws/src/movenao/launch/movenao.launch
```

#### To demo a node:
1. Build
2. Launch Q-Bot (Or any other package that you want to test)
3. In the terminal do:
```sh
~/scripts/testcnc.sh # Demo CNC node
```
Replace "cnc" by your desired node name.

#### Codes used
##### sys_state
```sh
0: Just woke up; RESET state
1: Activated; ready to work
10: Navigating
11: Reached the patient
20: Able to start question
21: Conducting questionnaire
22: Finished

100: Emergency Abort State
203: Taking a break
204: Contacting Nurse
```

##### NLPRes.res_type
```sh
0: Here’s your answer
1: Got gibberish, please ask again
2: Wasn’t expecting an answer, and got gibberish
3: Request for a break
4: Request for a nurse
```

#### Available postures:
```sh
['Crouch', 'LyingBack', 'LyingBelly', 'Sit', 'SitOnChair', 'SitRelax', 'Stand', 'StandInit', 'StandZero']
```

### Todos

 - Sze: publish sys_state to CNC_2_SPC every time it is changed (done)
 - Sze: subscribe to NLP_2_CNC topic	(done)
 - Guang: Code a minimalistic speech-processing node
 - Alex, Hanyi: Code a minimalistic naigation node

License
----

MIT

