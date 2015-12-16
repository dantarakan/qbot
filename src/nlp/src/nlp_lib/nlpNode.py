#!/usr/bin/env python

import rospy
from StringIO import StringIO
import pycurl, urllib, json, pprint, time, datetime
from nlp.msg import SpcCmd
from nlp.msg import SpcNLP
from nlp.msg import NLPRes
from nlp.msg import CncStatus
from std_msgs.msg import String

CONFIDENCE_VALUE = 0.5

class _Constants:
    questionDict = {'Hello I am QBot, what is your name?': 'name', 'How old are you?': 'age', 'How many hours did you sleep last night?': 'sleep', 'What is the last thing you ate, and when?': 'ate', 'What is the last thing you drunk, and when?': 'drunk', 'Are you currently pregnant?': 'yn', 'Do you have a sickle cell disease?': 'yn', 'Are you wearing any metal jewelry?': 'yn', 'Do you have any loose teeth, caps or crowns?': 'yn', 'Are you wearing glasses or contact lenses?': 'yn', 'Did pee recently?': 'yn', 'Do you have any prosthesis?': 'yn', 'Did you ever have any exposure to the mad cow disease?': 'yn', 'How often do find it hard to wind down?': 'frequency', 'How often aware of dryness in your mouth?': 'frequency', 'How often do you find that you cannot seem to experience any positive feeling at all?': 'frequency', 'How do you rate the level of service?': 'quality'}
    SpcNLP_topic = "SPC_2_NLP"
    NLPCnc_topic = "NLP_2_CNC"
    CncSpc_topic = "CNC_2_SPC"
    CncStatus_topic = "CNC_STATUS"

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
			self.setQuestion)

		self.__CncStatus_sub = rospy.Subscriber(
			_Constants.CncStatus_topic,
			CncStatus,
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
		    expected = ''
		    try:
		        expected = _Constants.questionDict[self.question]
		    except:
				try:
					expected = _Constants.questionDict[self.question[27:]]
				except:
					if "ready to start" in self.question:
						expected = 'yn'
					elif self.sys_state == 21:
						response.res_type = 1
					else:
						response.res_type = 2
		    rospy.logwarn(expected)
		    answer = self.getAnswerWIT(msg.response)
		    if not answer:
				if self.sys_state == 21:
					response.res_type = 1
				else:
					response.res_type = 2
		    else:
				readyAnswer = self.prepareAnswer(answer)
				rospy.logwarn(readyAnswer)

				if(readyAnswer == 'error' or not readyAnswer):
					if self.sys_state == 21:
						response.res_type = 1
					else:
						response.res_type = 2
				elif(readyAnswer[0] == 'ate' and (expected == 'ate' or expected == 'drunk')):
					if expected == 'ate':
					    foodResp = ''
					    for index in range(len(readyAnswer[1])):
					        foodResp += readyAnswer[1][index]
					        foodResp += ', '
					        try:
					            timeTemp = str(readyAnswer[3][len(readyAnswer[3])-index-1])[:19]
					            foodResp += timeTemp.replace('T', ' at ')
					            foodResp += ', '
					        except:
					            pass
					    response.response = foodResp[:(len(foodResp)-2)]
					    response.res_type = 0
					else:
					    drinkResp = ''
					    for index in range(len(readyAnswer[2])):
					        drinkResp += readyAnswer[2][index]
					        drinkResp += ', '
					        try:
					            timeTemp = str(readyAnswer[3][len(readyAnswer[3])-index-1])[:19]
					            drinkResp += timeTemp.replace('T', ' at ')
					            drinkResp += ', '
					        except:
					            pass
					    response.response = drinkResp[:(len(drinkResp)-2)]
					    response.res_type = 0
				elif(readyAnswer[0] == expected or ((readyAnswer[0] == 'yes' or readyAnswer[0] == 'no') and expected == 'yn')):
					if readyAnswer[0] == 'name' and (readyAnswer[1].lower() == 'Janice'.lower() or readyAnswer[1].lower() == 'Jonas'.lower() or readyAnswer[1].lower() == 'Janet'.lower() or readyAnswer[1].lower() == 'Alice'.lower()):
						response.response = 'Yannis'
					elif readyAnswer[0] == 'name' and (readyAnswer[1] == 'z'):
						response.response = 'Zee'
					else:
						response.response = str(readyAnswer[1])
					response.res_type = 0
				elif(readyAnswer[0] == 'number' and (expected == 'age' or expected == 'sleep')):
					response.response = str(readyAnswer[1])
					response.res_type = 0
				elif(readyAnswer[0] == 'duration' and expected == 'sleep'):
					response.response = str(readyAnswer[1])
					response.res_type = 0
				else:
					response.res_type = 1

		rospy.logwarn("NLP sending response: %s  type: %d\n", response.response, response.res_type)
		self.__pub.publish(response)
		

    def setQuestion(self, msg):
		# Capture system state and current question
		self.question = msg.question;
		rospy.logwarn("NLP received from CNC:\n qustion: %s\n", self.question)

    def setState(self, msg):
		# Capture system state and current question
		self.sys_state = msg.sys_state;
		#rospy.logwarn("NLP received from CNC:\n state: %d\n", self.sys_state)

    def getAnswerWIT(self, response):
		response = urllib.quote(response)

		c = pycurl.Curl()

		storage = StringIO()

		c.setopt(c.URL, "https://api.wit.ai/message?v=20151201&q=" + response)
		c.setopt(c.HTTPHEADER, ['Authorization: Bearer ZENOTRIDSMPNDKSMH33QGZZPREYJX5K5'])
		c.setopt(c.WRITEFUNCTION, storage.write)
		c.perform()
		c.close()

		try:
			resultJSON = json.loads(storage.getvalue())
			if (float(resultJSON['outcomes'][0]['confidence']) > CONFIDENCE_VALUE):
				return resultJSON['outcomes'][0]
			else:
				return ""
		except:
			return ""

    def prepareAnswer(self, answer):
		rospy.logwarn("Answer:")
		rospy.logwarn(answer)
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
				record.append(entities['name'][0]['value'])
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
					#time1 = datetime.datetime.strptime(item['value'], "%Y-%m-%dT%H:%M:%S.000-08:00").timetuple()
					#2015-12-16T18:00:00.000-08:00
					record[3].append(item['value'])
		elif(intent == 'yes'):
			record.append('yes')
			record.append('yes')
		elif(intent == 'deny'):
			record.append('no')
			record.append('no')
		elif(intent == 'never'):
			record.append('frequency')
			record.append('never')
		elif(intent == 'sometimes'):
			record.append('frequency')
			record.append('sometimes')
		elif(intent == 'always'):
			record.append('frequency')
			record.append('always')
		elif(intent == 'bad'):
			record.append('quality')
			record.append('bad')
		elif(intent == 'average'):
			record.append('quality')
			record.append('average')
		elif(intent == 'excellent'):
			record.append('quality')
			record.append('excellent')
		elif(intent == 'sleep'):
			record.append('sleep')
			datetimes = []
			if 'duration' in entities:
				record.append(entities['duration'][0]['value'])
			elif 'datetime' in entities:
				for item in entities['datetime']:
					datetimes.append(item['value'])
				rospy.logwarn(datetimes)
				timeList = list(set(datetimes))
				rospy.logwarn(timeList)
				filteredTimeList = []
				for i in range(len(timeList)-1):
					if self.countDiffs(timeList[i], timeList[i+1]) > 1 or self.diffG1(int(timeList[i][8:9]), int(timeList[i+1][8:9])):
						filteredTimeList.append(timeList[i])
					if i == len(timeList)-2:
						filteredTimeList.append(timeList[i+1])
				rospy.logwarn(filteredTimeList)
				if len(filteredTimeList) == 2:
					# Calc the difference
					convTime1 = datetime.datetime.strptime(filteredTimeList[0], "%Y-%m-%dT%H:%M:%S.000Z").timetuple()
					convTime2 = datetime.datetime.strptime(filteredTimeList[1], "%Y-%m-%dT%H:%M:%S.000Z").timetuple()
					difference = abs((time.mktime(convTime1) - time.mktime(convTime2))) / 3600
					if difference > 12:
						difference = 24 - difference
					record.append(difference)
				else:
					return 'error'
		elif(intent == 'number'):
			record.append('number')
			if 'number' in entities:
				record.append(entities['number'][0]['value'])
		elif(intent == 'duration'):
			record.append('duration')
			if 'duration' in entities:
				record.append(entities['duration'][0]['value'])

		return record
		
    def diffG1(self, a, b):
        diff = abs(a-b)
        if diff == 1 or diff == 29 or diff == 30:
            return False
        else:
            return True
            
    def countDiffs(self, a, b):
        u=zip(a,b)
        counter = 0
        for i,j in u:
            if i!=j:
                counter += 1
        return counter
