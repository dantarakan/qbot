import speech_recognition as sr

# obtain path to "test.wav" in the same folder as this script

# use "test.wav" as the audio source


def real_time_recog():
	r = sr.Recognizer()
	with sr.Microphone() as source:
		audio = r.listen(source) # read the entire WAV file

	# recognize speech using Google Speech Recognition
	try:
		# for testing purposes, we're just using the default API key
		# to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
		# instead of `r.recognize_google(audio)` key='AIzaSyDJMVC1D6xIF96bVDxRkYivo-tPr_Ps0W4'
		s = r.recognize_google(audio)
		print("::::" + s)#, ))
		return s
	except sr.UnknownValueError:
		print(sr.UnknownValueError)
		print("????") # Google Speech Recognition could not understand audio
		return 'google speech recognition error'
	except sr.RequestError as e:
		print("Could not request results from Google Speech Recognition service; {0};;;;".format(e))
		return 'request error'
		
real_time_recog()
