#include <ros/ros.h>
#include <qbot/NavCmd.h>
#include <qbot/SpcCmd.h>
#include <qbot/NavRes.h>
#include <qbot/SpcRes.h>

#include <iostream>  //declaring variables
#include <string>
#include <fstream>
#include <vector>

const int bufferSize = 1000;
uint8_t sys_state = 0;
std::vector<std::string> questions;

class Command_Node{
private:
	ros::Publisher navPub_;
	ros::Publisher spcPub_;
	ros::Subscriber navSub_;
	ros::Subscriber spcSub_;
	uint8_t qnum;
	std::string response;
	//std::vector<std::string>& qnlist;
	
public:
	Command_Node(ros::NodeHandle& n){
		navPub_ = n.advertise<qbot::NavCmd>("/CNC_2_NAV", bufferSize);
		spcPub_ = n.advertise<qbot::SpcCmd>("/CNC_2_SPC", bufferSize);
		navSub_ = n.subscribe("/NAV_2_CNC", bufferSize, &Command_Node::navCallback, this);
		spcSub_ = n.subscribe("/SPC_2_CNC", bufferSize, &Command_Node::spcCallback, this);
		
		qnum=0;
		
		// TODO: insert logic to enter starting parameters like bed ward etc
		
	}
	
	void navCallback(const qbot::NavRes& navres){
		// TODO: insert process flow logic here
		
		//if not navigating or not reached
		if(navres.status!=1 && sys_state==0){
			qbot::NavCmd navcmd;
			navcmd.status = 1; // tell it to move
			navcmd.floor = 1;
			navcmd.room = 10;
			navcmd.bed = 5;
			navPub_.publish(navcmd);
			ROS_INFO("Start Navigation...");
			sys_state = 10;
		}
		else{
			qbot::NavCmd navcmd;
			navcmd.status = 0; // tell it to stop
			navPub_.publish(navcmd);
			ROS_INFO("Stop Navigation...");
			
			sys_state = 11;
		}
		
	}
	
	void spcCallback(const qbot::SpcRes& spcres){
		// TODO: insert process flow logic here
		if(spcres.status==0 && sys_state==11){
			qbot::SpcCmd spccmd;
			spccmd.question = "Hello, what is your name?";
			ROS_INFO("Introducing...\n");
			spcPub_.publish(spccmd);
			
			sys_state = 20; // 20: able to start question
		}
		else if(spcres.status==1 && sys_state==20){
			response = spcres.response;
			ROS_INFO("Reply: %s \n", response.c_str());
			
			qbot::SpcCmd spccmd;			
			spccmd.question = questions.at(qnum);
			ROS_INFO("QBot: %s ", questions[qnum].c_str());
			qnum++;
			spcPub_.publish(spccmd);
			
			sys_state = 21; // 21: In progress with questioning
		}
		else if(spcres.status==1 && sys_state==21){
			response = spcres.response;
			ROS_INFO("Reply: %s \n", response.c_str());
			
			if(qnum < (int)questions.size()){
				qbot::SpcCmd spccmd;			
				spccmd.question = questions.at(qnum);
				ROS_INFO("QBot: %s ", questions[qnum].c_str());
				qnum++;
				spcPub_.publish(spccmd);
			}
			else sys_state=22; // finished
		}
	}

};


int main(int argc, char** argv){

	std::string line;
	std::ofstream outfile;
    std::ifstream infile("/home/human/catkin_ws/src/qbot/src/questions.txt");
    
    if(infile.is_open())
    {	
    	while(!infile.eof())
    	{
    		std::getline(infile, line);
    		questions.push_back(line);
    	}
    }
    else{
    	std::cout << "Unable to open file" << std::endl;
    	exit(1);
    }
    
    infile.close();
    
    std::cout << "Size of questions: " << questions.size() << std::endl;
    

	ros::init(argc, argv, "command_module");
	ros::NodeHandle n;
	
	Command_Node cmdnode(n);
	
	ROS_INFO("Command Node Started");
	ros::spin();
	return 0;
}
