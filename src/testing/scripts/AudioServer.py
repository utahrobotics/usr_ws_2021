#!/usr/bin/env python

import pyaudio
import wave
import rospy
from testing.srv import audio,audioResponse
import time as tm
import os 
import errno

class AudioServer():
    
    def __init__(self):
	rospy.init_node('AudioServer')
	self.audio_service = rospy.Service("Audio", audio, self.sendAudio)
         #self.audio = some audio file
	print("Ready to respond")
    	rospy.spin()
    
    
    def getAudio(self, time):
        form_1 = pyaudio.paInt16 # 16-bit resolution
	chans = 1 # 1 channel
	samp_rate = 44100 # 44.1kHz sampling rate
	chunk = 4096 # 2^12 samples for buffer
	record_secs = time # seconds to record
	dev_index = 22 # device index found by p.get_device_info_by_index(ii)
	try:
		os.makedirs("/home/usr/recordings/")
	except OSError as e:
		if e.errno != errno.EEXIST:
			raise
	wav_output_filename = '/home/usr/recordings/'+str(tm.time())+'.wav' # name of .wav file

	audio = pyaudio.PyAudio() # create pyaudio instantiation

	# create pyaudio stream
	stream = audio.open(format = form_1,rate = samp_rate,channels = chans, \
                    input_device_index = dev_index,input = True, \
                    frames_per_buffer=chunk)
	print("recording")
	frames = []

	# loop through stream and append audio chunks to frame array
	for ii in range(0,int((samp_rate/chunk)*record_secs)):
	    data = stream.read(chunk)
	    frames.append(data)

	print("finished recording")

	# stop the stream, close it, and terminate the pyaudio instantiation
	stream.stop_stream()
	stream.close()
	audio.terminate()

	# save the audio frames as .wav file
	wavefile = wave.open(wav_output_filename,'wb')
	wavefile.setnchannels(chans)
	wavefile.setsampwidth(audio.get_sample_size(form_1))
	wavefile.setframerate(samp_rate)
	wavefile.writeframes(b''.join(frames))
	wavefile.close()
	return wav_output_filename

    def sendAudio(self,req):
	return audioResponse(self.getAudio(req.time))
	    
if __name__ == '__main__':
	audioServer = AudioServer()