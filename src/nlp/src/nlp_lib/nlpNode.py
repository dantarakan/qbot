#!/usr/bin/env python

import rospy
from StringIO import StringIO
import pycurl, urllib, json, pprint
from nlp.msg import SpcCmd
from nlp.msg import SpcNLP
from nlp.msg import NLPRes
from std_msgs.msg import String

CONFIDENCE_VALUE = 0.5

class _Constants:
    SpcNLP_topic = "SPC_2_NLP"
    NLPCnc_topic = "NLP_2_CNC"
    CncSpc_topic = "CNC_2_SPC"

class NLP:
    def __init__(self):
		self.sys_state = 0
		self.question = ""
		self.__pub = rospy.Publisher(_Constants.NLPCnc_topic, NLPRes, latch=True)

		self.__SpcNLP_sub = rospy.Subscriber(
			_Constants.SpcNLP_topic,
			SpcNLP,
			self.getMessage)

		self.__CncSpc_sub = rospy.Subscriber(
			_Constants.CncSpc_topic,
			SpcCmd,
			self.setState)
            
    def getMessage(self, msg):
		response = NLPRes()
		rospy.logwarn("NLP received response from SPC: %s\n", msg.response)
		if not msg.response:
			if self.sys_state == 21:
				response.res_type = 1
			else:
				response.res_type = 2
		elif 'break' in msg.response:
			response.res_type = 3
		elif 'nurse' in msg.response:
			response.res_type = 4
		else:
			response.response = self.getAnswer(msg.response)
			response.res_type = 0
		rospy.logwarn("NLP sending response: %s  type: %d\n", response.response, response.res_type)
		self.__pub.publish(response)
		

    def setState(self, msg):
		# Capture system state and current question
		self.sys_state = msg.sys_state;
		self.question = msg.question;
		rospy.logwarn("NLP received from CNC:\n state: %d \n qustion: %s\n", self.sys_state, self.question)

    def getAnswer(self, response):
	response = urllib.quote(response)
	pp = pprint.PrettyPrinter(indent=4)

	c = pycurl.Curl()

	storage = StringIO()

	c.setopt(c.URL, "https://api.wit.ai/message?v=20151201&q=" + response)
	c.setopt(c.HTTPHEADER, ['Authorization: Bearer ZENOTRIDSMPNDKSMH33QGZZPREYJX5K5'])
	c.setopt(c.WRITEFUNCTION, storage.write)
	c.perform()
	c.close()

	try:
		resultJSON = json.loads(storage.getvalue())
		#pp.pprint(resultJSON)
		rospy.logwarn(resultJSON['outcomes'])
		if (float(resultJSON['outcomes'][0]['confidence']) > CONFIDENCE_VALUE):
			
			return resultJSON['outcomes'][0]['intent']
		else:
			return ""
	except:
		return ""

