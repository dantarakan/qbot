#include <ros/ros.h>
#include <std_msgs/String.h>
#include <qbot/NavCmd.h>
#include <qbot/SpcCmd.h>
#include <qbot/NavRes.h>
#include <qbot/SpcRes.h>
#include <qbot/NLPRes.h>
#include <qbot/GuiRes.h>
#include <qbot/CncStatus.h>

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
	ros::Publisher movenaoPub_;
	ros::Subscriber navSub_;
	//ros::Subscriber spcSub_;
	ros::Subscriber nlpSub_;
	ros::Subscriber guiSub_;
	uint8_t qnum;
	std::string response;
	//std::vector<std::string>& qnlist;
	
public:
	Command_Node(ros::NodeHandle& n){
		navPub_ = n.advertise<qbot::NavCmd>("/CNC_2_NAV", bufferSize);
		spcPub_ = n.advertise<qbot::SpcCmd>("/CNC_2_SPC", bufferSize);
		movenaoPub_ = n.advertise<std_msgs::String>("/posture", bufferSize);
		navSub_ = n.subscribe("/NAV_2_CNC", bufferSize, &Command_Node::navCallback, this);
		//spcSub_ = n.subscribe("/SPC_2_CNC", bufferSize, &Command_Node::spcCallback, this);
		nlpSub_ = n.subscribe("/NLP_2_CNC", bufferSize, &Command_Node::nlpCallback, this);
		guiSub_ = n.subscribe("/GUI_2_CNC", bufferSize, &Command_Node::guiCallback, this);
		
		qnum=0;
		
		// TODO: insert logic to enter starting parameters like bed ward etc
		
	}
	
	void guiCallback(const qbot::GuiRes& guires){
		
		if(guires.cmdcode==100){
			qbot::NavCmd navcmd;
			navcmd.status = 0; // tell it to stop
			navPub_.publish(navcmd);
			
			qbot::SpcCmd spccmd;
			spccmd.question = "Hang on, something has gone wrong. Sorry about that.";
			spcPub_.publish(spccmd);
		
			ROS_INFO("QBot Emergency ABORT");
			sys_state = 100; // ALERT OPERATOR
		}
		else if(guires.cmdcode==101){
			ROS_INFO("QBot RESET");
			sys_state = 0;
			qnum = 0;
			
			std_msgs::String msg;
			msg.data = "Sit";
			movenaoPub_.publish(msg);
			ros::Duration(5).sleep();
			
		}
		else if(guires.cmdcode==1 && sys_state==0){
			ROS_INFO("QBot Activated");
			sys_state = 1;
		}
		
		
		// TODO: vvv confirm what triggers the introduction
		if(guires.cmdcode==200 && sys_state==11){
		
		    std_msgs::String msg;
			msg.data = "Stand";
			movenaoPub_.publish(msg);
			
			ros::Duration(10).sleep();
		
			qbot::SpcCmd spccmd;
			//spccmd.question = "Hello I am QBot, what is your name?";
			spccmd.question = questions.at(qnum);
			spcPub_.publish(spccmd);
			
			ROS_INFO("Introducing...\n");
			
			sys_state = 20; // 20: able to start question
		}
	
	}
	
	void navCallback(const qbot::NavRes& navres){
		
		//if not navigating or not reached
		if(navres.status!=1 && sys_state==1){
			qbot::NavCmd navcmd;
			navcmd.status = 1; // tell it to move
			navcmd.floor = 1;
			navcmd.room = 10;
			navcmd.bed = 5;
			navPub_.publish(navcmd);
			ROS_INFO("Start Navigation...");
			sys_state = 10;
		}
		else if(sys_state==10){
			qbot::NavCmd navcmd;
			navcmd.status = 0; // tell it to stop
			navPub_.publish(navcmd);
			ROS_INFO("Stop Navigation...");
			
			sys_state = 11;
		}
		
	}
	
	void nlpCallback(const qbot::NLPRes& nlpres){
		
		
		if(nlpres.res_type==0 && sys_state==20){
			response = nlpres.response;
			ROS_INFO("Reply: %s \n", response.c_str());
			
			qbot::SpcCmd spccmd;
			
			spccmd.question = "Nice to meet you " + response;
			spcPub_.publish(spccmd);
			ROS_INFO("QBot: Nice to meet you" );
					
			spccmd.question = questions.at(qnum);
			spcPub_.publish(spccmd);
			ROS_INFO("QBot: %s ", questions[qnum].c_str());
			
			sys_state = 21; // 21: In progress with questioning
		}
		else if(nlpres.res_type==0 && sys_state==21){
			response = nlpres.response;
			ROS_INFO("Reply: %s \n", response.c_str());
			
			qnum++;
			
			if(qnum < (int)questions.size()){	
				qbot::SpcCmd spccmd;			
				spccmd.question = questions.at(qnum);
				spcPub_.publish(spccmd);
				ROS_INFO("QBot: %s ", questions[qnum].c_str());
			}
			else sys_state=22; // finished
		}
		else if(nlpres.res_type==1 && sys_state==21){
			response = nlpres.response;
			ROS_INFO("Reply: %s \n", response.c_str());
			
			// Got gibberish; ask same question again
			qbot::SpcCmd spccmd;			
			spccmd.question = "Sorry I didn't catch that, " + questions.at(qnum);
			spcPub_.publish(spccmd);
			ROS_INFO("QBot: %s ", questions[qnum].c_str());
		}
		
		
		//Other responses
		if(nlpres.res_type==3){
			qbot::SpcCmd spccmd;
			spccmd.question = "Okay, we will take a short break";
			spcPub_.publish(spccmd);
			
			ROS_INFO("Taking break...\n");
			
			sys_state = 203; // 203: able to start question
		}
		if(nlpres.res_type==4){
			qbot::SpcCmd spccmd;
			spccmd.question = "Hang on, I will contact the nurse";
			spcPub_.publish(spccmd);
			
			ROS_INFO("Contact Nurse...\n");
			
			sys_state = 204; // 203: able to start question
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
	
	ros::Publisher cncPub_ = n.advertise<qbot::CncStatus>("/CNC_STATUS", bufferSize);
	ros::Rate loop_rate(10);
	
	while(ros::ok()){
	
		qbot::CncStatus cncstatus;
		cncstatus.sys_state = sys_state;
		cncPub_.publish(cncstatus);
	
		ros::spinOnce();
		
		loop_rate.sleep();
	}
	return 0;
}











