#include <ros/ros.h>
#include <qbot/SpcCmd.h>
#include <qbot/SpcNLP.h>
#include <qbot/NLPRes.h>

#include <iostream>  //declaring variables
#include <string>
#include <fstream>
#include <vector>

const int bufferSize = 1000;
uint8_t sys_state = 0;
std::string question = "";

class NLP_Node{
private:
	ros::Publisher nlpPub_; // NLP to CNC - NLPRes.msg
	ros::Subscriber cncSub_; // CNC to SPC - SpcCmd.msg
	ros::Subscriber spcSub_; // SPC to NLP - SpcNLP.msg
	
public:
	NLP_Node(ros::NodeHandle& n){
		nlpPub_ = n.advertise<qbot::NLPRes>("/NLP_2_CNC", bufferSize);
		cncSub_ = n.subscribe("/CNC_2_SPC", bufferSize, &NLP_Node::cncCallback, this);
		spcSub_ = n.subscribe("/SPC_2_NLP", bufferSize, &NLP_Node::spcCallback, this);
	}
	
	std::string getAnswer(std::string response){
		return "6";
	}
	
	// CNC messaged SPC
	void cncCallback(const qbot::SpcCmd& spccmd){
		// Capture system state and current question
		sys_state = spccmd.sys_state;
		question = spccmd.question;
		ROS_INFO("NLP received from CNC:\n state: %d \n qustion: %s\n", sys_state, question.c_str());
	}
	
	// SPC messaged NLP
	void spcCallback(const qbot::SpcNLP& spcnlp){
		qbot::NLPRes nlpres; // Init response to CNC
		
		ROS_INFO("NLP received response from SPC: %s\n", spcnlp.response.c_str());
		// If response is empty, return an error
		if(spcnlp.response.empty())
		{
			nlpres.res_type = 1;
		}
		else
		{
			// Try to match response to a request for a break
			if(spcnlp.response.find("break") != std::string::npos)
			{
				nlpres.res_type = 3;
			}
			// Try to match response to a request for a nurse
			else if(spcnlp.response.find("nurse") != std::string::npos)
			{
				nlpres.res_type = 4;
			}
			// If expecting a response
			else if(sys_state == 21)
			{
				// Get formatted answer
				nlpres.response = getAnswer(spcnlp.response);
				
				// If couldn't get an answer, request to ask again
				if(nlpres.response.empty())
				{
					nlpres.res_type = 1;
				}
				else
				{
					nlpres.res_type = 0;
				}
			}
		}
		
		// Publish the response
		ROS_INFO("NLP sending response: %s  type: %d\n", nlpres.response.c_str(), nlpres.res_type);
		nlpPub_.publish(nlpres);
	}
};


int main(int argc, char** argv){
	ros::init(argc, argv, "nlp_node");
	ros::NodeHandle n;
	
	NLP_Node nlpnode(n);
	
	ROS_INFO("NLP Node Started");
	ros::spin();
	return 0;
}
