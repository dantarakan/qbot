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
#include <sstream> 
#include <vector>
#include <time.h>
#include <stdlib.h>

const int bufferSize = 1000;
uint8_t sys_state = 0;
std::vector<std::string> questions;

class Command_Node{
private:
	ros::Publisher navPub_;
	ros::Publisher spcPub_;
	ros::Publisher movenaoPub_;
	ros::Publisher facetrackPub_;
	ros::Subscriber navSub_;
	ros::Subscriber nlpSub_;
	ros::Subscriber guiSub_;
	uint8_t qnum, trynum;
	std::string response, filename;
	std::ofstream outfile, timingoutfile, resoutfile;
	ros::Time tCycles;
	double t1;
	std::stringstream convert;

	//std::vector<std::string>& qnlist;
	
public:
	Command_Node(ros::NodeHandle& n){
		navPub_ = n.advertise<qbot::NavCmd>("/CNC_2_NAV", bufferSize);
		spcPub_ = n.advertise<qbot::SpcCmd>("/CNC_2_SPC", bufferSize);
		movenaoPub_ = n.advertise<std_msgs::String>("/posture", bufferSize);
		facetrackPub_ = n.advertise<std_msgs::String>("/CNC_2_FACETRACK", bufferSize);
		navSub_ = n.subscribe("/NAV_2_CNC", bufferSize, &Command_Node::navCallback, this);
		nlpSub_ = n.subscribe("/NLP_2_CNC", bufferSize, &Command_Node::nlpCallback, this);
		guiSub_ = n.subscribe("/GUI_2_CNC", bufferSize, &Command_Node::guiCallback, this);
		
		qnum=0;
		
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
			
			if(outfile.is_open()){
			    outfile.close();
			}
			if(timingoutfile.is_open()){
			    timingoutfile.close();
			}
			if(resoutfile.is_open()){
			    resoutfile.close();
			}

			srand(time(NULL));
		    convert << rand();
		    filename = "/home/human/catkin_ws/src/qbot/results/" + convert.str() + ".txt";
		    outfile.open(filename.c_str());
		    
		    timingoutfile.open("/home/human/catkin_ws/src/qbot/results/timing.txt", std::ios_base::app);
		    timingoutfile << std::endl;
		    
		    resoutfile.open("/home/human/catkin_ws/src/qbot/results/responses.txt", std::ios_base::app);
		    resoutfile << std::endl;
			
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
		if((guires.cmdcode==200 && sys_state==11) || (guires.cmdcode==250) ){
			
			/*
		    std_msgs::String msg;
			msg.data = "Start Face Tracking";
			facetrackPub_.publish(msg);
			
			ros::Duration(10).sleep();
			*/
			
			sys_state = 20; // 20: able to start question
			
			qbot::SpcCmd spccmd;
			spccmd.question = "Just to let you know, when my ears are lit up, it means that I am listening";
			spccmd.spc_state = 100;
			spcPub_.publish(spccmd);
			
			ros::Duration(1).sleep();
		
			qbot::SpcCmd spccmd2;
			spccmd2.question = "Hello I am QBot, what is your name?";
			spccmd2.spc_state = 0;
			spcPub_.publish(spccmd2);
			
			ROS_INFO("Introducing...\n");

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
		
		
		if(sys_state==20){
		
		    sys_state = 21; // 21: Checking to start
		    
			if(nlpres.res_type==0){
				response = nlpres.response;
				ROS_INFO("Reply: %s \n", response.c_str());
			
				qbot::SpcCmd spccmd;
				spccmd.question = "Nice to meet you " + response + ". Are you ready to start?";
				spcPub_.publish(spccmd);
				ROS_INFO("QBot: Nice to meet you [Name]" );
			}
			else{
				qbot::SpcCmd spccmd;
				spccmd.question = "Nice to meet you. Are you ready to start?";
				spcPub_.publish(spccmd);
				ROS_INFO("QBot: Nice to meet you" );
			}
			
		}
		else if(sys_state==21){
			if(nlpres.res_type==0){
				response = nlpres.response;
				ROS_INFO("Reply: %s \n", response.c_str());
				
				if(response.compare("yes") == 0){
				    sys_state = 22; // 22 In progress with questioning
				    
				    tCycles = ros::Time::now();
	    
					qbot::SpcCmd spccmd;			
					spccmd.question = questions.at(qnum);
					spcPub_.publish(spccmd);
					ROS_INFO("QBot: %s ", questions[qnum].c_str());
					
					outfile << "Question: " << questions.at(qnum) << std::endl;
				}
				else{
					qbot::SpcCmd spccmd;
					spccmd.question = "Are you ready to start?";
					spcPub_.publish(spccmd);
					ROS_INFO("QBot: Are you ready to start?" );
				}
			}
			else{
				qbot::SpcCmd spccmd;
				spccmd.question = "Are you ready to start?";
				spcPub_.publish(spccmd);
				ROS_INFO("QBot: Are you ready to start?" );
			}
		}
		else if(sys_state==22 && (nlpres.res_type==0 || trynum>=1 )){
			response = nlpres.response;
			ROS_INFO("Reply: %s \n", response.c_str());

			t1 = (ros::Time::now().toSec() - tCycles.toSec());
			// Record t1
			ROS_INFO("Time: %f \n", t1);
			
			qnum++;
			trynum = 0;
			
			outfile << "Reply: " << response << std::endl;
			timingoutfile << " ," << t1;
			resoutfile << " ," << response;
			
			if(qnum < ((int)questions.size()-1)){
			
			    tCycles = ros::Time::now();	
				qbot::SpcCmd spccmd;			
				spccmd.question = questions.at(qnum);
				spcPub_.publish(spccmd);
				ROS_INFO("QBot: %s ", questions[qnum].c_str());
				
				outfile << "Question: " << questions.at(qnum) << std::endl;
			}
			else{
			    outfile << "-----===== END =====-----" << std::endl;
			    outfile.close();
			    
			    timingoutfile.close();
			    resoutfile.close();
			    
			    sys_state=30; // finished
			    
			    qbot::SpcCmd spccmd;
				spccmd.question = "Thank you for your time. Merry Christmas and Happy Holidays. QBot out.";
				spccmd.spc_state = 100;
				spcPub_.publish(spccmd);
				ROS_INFO("QBot: Thank you for your time. QBot out." );
			}
		}
		else if((nlpres.res_type==1||nlpres.res_type==2) && sys_state==22){
			response = nlpres.response;
			ROS_INFO("Reply: %s \n", response.c_str());
			
			// Got gibberish; ask same question again
			
			trynum++;
			
			qbot::SpcCmd spccmd;			
			spccmd.question = "Sorry I didn't catch that, " + questions.at(qnum);
			spcPub_.publish(spccmd);
			ROS_INFO("QBot: %s ", questions[qnum].c_str());
		}
		else if((nlpres.res_type==5) && sys_state==22){
			ROS_INFO("Subject didn't understand the question.\n");
			
			// Subject didn't understand the question; ask same question again
			qbot::SpcCmd spccmd;			
			spccmd.question = "No worries, I will repeat the question, " + questions.at(qnum);
			spcPub_.publish(spccmd);
			ROS_INFO("QBot: %s ", questions[qnum].c_str());
		}
		else if(sys_state==30){
			/*
			std_msgs::String msg;
			msg.data = "Sit";
			movenaoPub_.publish(msg);
			
			ros::Duration(5).sleep();*/
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











