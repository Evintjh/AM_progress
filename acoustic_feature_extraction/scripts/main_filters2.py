#!/usr/bin/env python


'''
This is the noise filtering node
using scipy fft function to select the frequency, use a bandpass filter
'''


import numpy as np
from scipy.fftpack import rfft, rfftfreq, irfft, fft, ifft, fftfreq
import rospy
from scipy import signal

from std_msgs.msg import (
    Bool
)
from acoustic_monitoring_msgs.msg import (
    AudioDataStamped,										
)
#from scipy.signal import butter, lfilter




	        
class FilterNode:

    def __init__(self):
        rospy.init_node("main_filters")
        self._min_freq = rospy.get_param('~min_freq', 0)
        self._max_freq = rospy.get_param('~max_freq', 500)									
        self._sample_rate = rospy.get_param('~sample_rate', 16000)
        self._frame_duration = rospy.get_param('~frame_duration', .010) #10 ms
        self._min_freq_scaled = int(self._min_freq * self._frame_duration)
        self._max_freq_scaled = int(self._max_freq * self._frame_duration)

        self._signal_pub = rospy.Publisher('filteredAudioStamped', AudioDataStamped, queue_size=5) 
        rospy.Subscriber('audioStamped', AudioDataStamped, self._audio_cb, queue_size=5)
        



    def _audio_cb(self, msg):	
    	
        nft = self._sample_rate*4
                

                    # convert the audio to numpy array
        audio_data = msg.data
        audio_data_numpy = np.frombuffer(audio_data, dtype=np.int16)		
        origFFT = fft(audio_data_numpy)		
                
        #amplitude of each frequency range
        nobVal = [10,10,10,10,5,5,5,5]
                
        Gain=1; #gain
                
        #normalizing the nobs from range of 0 to 10 to range of 0 to 1 by diving each nob value by 10.
        nobVal[:] = [i/10 for i in nobVal]
                
        #frequency array
        freq = [32,64,128,256,512,1000,2000,3000]		#array is used for bin conversion? supposed to adjust cutoff freq according to this?
                
        #initializing a bin array
        bin = [0] * 8
                
        for i in range (0,8):
            bin[i]= int((freq[i]*nft)/self._sample_rate)

        #scaling the frequencies
        freq[:] = [i*self._sample_rate/nft for i in (0,nft-1)]			#not needed actually in our case
                
                
        #defing the end of the spectrum since it will be required due to symmetry.
        #end = int(X[-1])
                
        #equalising according to input
        for i in range (0,7):
            origFFT[bin[i]:bin[i+1]]=origFFT[bin[i]:bin[i+1]]*nobVal[i]*Gain



        # create the fittered wave to send to the vad
        audio_data_numpy_new = np.real(ifft(origFFT)).astype('int16')			#ifft operation on modified audio,
        audio_data_filtered = audio_data_numpy_new.tobytes()

        # create the message with the filtered audio to be sent to the VAD
        response = AudioDataStamped()
        response.header = msg.header
        response.data = audio_data_filtered
        # Add check for more graceful shutdown and don't publish if at the end.
        if not rospy.is_shutdown():											####
            self._signal_pub.publish(response)








if __name__ == '__main__':
    #rospy.init_node("main_filters", anonymous = True)											
    FilterNode()
    rospy.spin()
