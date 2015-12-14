#!/usr/bin/env python

import rospy
from StringIO import StringIO
import pycurl, urllib, json, pprint, time, datetime
from nlp.msg import SpcCmd
from nlp.msg import SpcNLP
from nlp.msg import NLPRes
from std_msgs.msg import String

CONFIDENCE_VALUE = 0.5

class _Constants:
	questionDict = {'How are you?': 'state', 'What is your name?': 'name', 'How old are you?': 'age', 'How many hours did you sleep last night?': 'sleep', 'What is the last thing you ate, and when?': 'ate', 'What is the last thing you drunk, and when?': 'drunk', 'Are you currently pregnant?': 'yn', 'Do you have a sickle cell disease?': 'yn', 'Are you wearing any metal jewelry?': 'yn', 'Do you have any loose teeth, caps or crowns?': 'yn', 'Are you wearing glasses or contact lenses?': 'yn', 'Did pee recently?': 'yn', 'Do you have any prosthesis?': 'yn', 'Did you ever have any exposure to the mad cow disease?': 'yn', 'How often do find it hard to wind down?': 'frequency', 'How often aware of dryness in your mouth?': 'frequency', 'How often do you find that you cannot seem to experience any positive feeling at all?': 'frequency', 'How do you rate the level of service?': 'quality'};
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
			answer = self.getAnswerWIT(msg.response)
			readyAnswer = self.prepareAnswer(answer)
			rospy.logwarn(readyAnswer)

			expected = questionDict[self.question]

			if(readyAnswer == 'error' || not readyAnswer):
				if self.sys_state == 21:
					response.res_type = 1
				else:
					response.res_type = 2
			elif(readyAnswer[0] == 'ate' && (expected == 'ate' || expected == 'drunk')):
				
			elif(readyAnswer[0] == expected || ((readyAnswer[0] == 'yes' || readyAnswer[0] == 'no') && expected == 'yn'))
				response.response = readyAnswer[0]
				response.res_type = 0


			readyAnswerJSON = json.encode(readyAnswer)
			response.response = readyAnswerJSON
			response.res_type = 0


		rospy.logwarn("NLP sending response: %s  type: %d\n", response.response, response.res_type)
		self.__pub.publish(response)
		

    def setState(self, msg):
		# Capture system state and current question
		self.sys_state = msg.sys_state;
		self.question = msg.question;
		rospy.logwarn("NLP received from CNC:\n state: %d \n qustion: %s\n", self.sys_state, self.question)

    def getAnswerWIT(self, response):
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
				return resultJSON['outcomes'][0]
			else:
				return ""
		except:
			return ""

    def prepareAnswer(self, answer):
		intent = answer['intent']
		entities = answer['entities']
		record = []
		#rospy.logwarn(entities)
		if(intent == 'am'):
			if 'age_of_person' in entities:
				record.append('age')
				record.append(entities['age_of_person'][0]['value'])
			elif 'name' in entities:
				record.append('name')
				record.append(entities['age_of_person'][0]['value'])
		elif(intent == 'ate'):
			record.append('ate')
			record.append([])
			record.append([])
			record.append([])
			if 'food' in entities:
				for item in entities['food']:
					record[1].append(item['value'])
			if 'drink' in entities:
				for item in entities['drink']:
					record[2].append(item['value'])
			if 'datetime' in entities:
				for item in entities['datetime']:
					time1 = datetime.datetime.strptime(item['value'], "%d %B %Y at %H:%M:%S %Z").timetuple()
					record[3].append(time1)
		elif(intent == 'yes'):
			record.append('yes')
		elif(intent == 'deny'):
			record.append('no')
		elif(intent == 'sleep'):
			record.append('sleep')
			datetime = []
			if 'duration' in entities:
				record.append(entities['duration'][0]['value'])
			elif 'datetime' in entities:
				for item in entities['datetime']:
					datetime.append(item['value'])
			if datetime.count == 2:
				# Calc the difference
				convTime1 = datetime.datetime.strptime(datetime[0], "%d %B %Y at %H:%M:%S %Z").timetuple()
				convTime2 = datetime.datetime.strptime(datetime[1], "%d %B %Y at %H:%M:%S %Z").timetuple()
				difference = abs((time.mktime(convTime1) - time.mktime(convTime2)) / 216000)
				record.append(difference)
			else:
				return 'error'

		return record
