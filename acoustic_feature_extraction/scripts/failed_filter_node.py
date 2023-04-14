#!/usr/bin/env python


'''
This is the noise filtering node
using scipy fft function to select the frequency, use a bandpass filter
'''


import numpy as np
from matplotlib import pyplot as plt
from scipy.fftpack import rfft, rfftfreq, irfft, fft, ifft, fftfreq
import rospy
from scipy import signal
from scipy.signal import butter, lfilter

from std_msgs.msg import (
    Bool
)
from acoustic_monitoring_msgs.msg import (
    AudioDataStamped,										###is this the sound source?
)
from scipy.signal import butter, lfilter




	        
rospy.init_node('filter_node3', anonymous=True)				#### 
rospy.Subscriber('audioStamped', AudioDataStamped, main, queue_size=5)
signal_published = rospy.Publisher('filteredAudioStamped', AudioDataStamped, queue_size=5) 			####  isit we have to publish back to the same topic as that of subscribed?
    
    



def bandpass_filter(data, lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs													#shudn't this be 2?
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='bandpass')
    filtered = lfilter(b, a, data)
    return filtered


def equalizer_10band (data, fs, gain1=0, gain2=0, gain3=0, gain4=0):
    band1 = bandpass_filter(data, 0, min_freq_scaled, fs, order=2)* 10**(gain1/20)
    band2 = bandpass_filter(data, min_freq_scaled+1, 80, fs, order=3)*10**(gain2/20)
    band3 = bandpass_filter(data, 81, 160 - max_freq_scaled, fs, order=3)*10**(gain3/20)								
    band4 = bandpass_filter(data, 160 - max_freq_scaled + 1, 160, fs, order=3)* 10**(gain4/20)
    signal = band1 + band2 + band3 + band4 
    return signal
    


min_freq = rospy.get_param('~min_freq', 0)
max_freq = rospy.get_param('~max_freq', 500)							#freq ranges and sampling freq
sample_rate = rospy.get_param('~sample_rate', 16000)
frame_duration = rospy.get_param('~frame_duration', .010) #10 ms
min_freq_scaled = int(min_freq * frame_duration)
max_freq_scaled = int(max_freq * frame_duration)




def main(msg):	

	# convert the audio to numpy array
    audio_data = msg.data
    audio_data_numpy = np.frombuffer(audio_data, dtype=np.int16)				#original data to be played with



	#appying equalizer											#filtered signal, i need to read the fre
    equalized = equalizer_10band(audio_data_numpy, sample_rate, 0,0,0,0)



	#compute ftt of filtered signal
    Y = fft(equalized)


        # create the fittered wave to send to the vad
    audio_data_numpy_new = np.real(ifft(Y)).astype('int16')
    audio_data_filtered = audio_data_numpy_new.tobytes()

        # create the message with the filtered audio to be sent to the VAD
    response = AudioDataStamped()
    response.header = msg.header
    response.data = audio_data_filtered
        # Add check for more graceful shutdown and don't publish if at the end.
    if not rospy.is_shutdown():											####
        signal_published.publish(response)








if __name__ == '__main__':
    #rospy.init_node("filter_node3", anonymous = True)											###???
    main(msg)
    rospy.spin()
