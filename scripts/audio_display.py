#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import rosservice
import cv2
import numpy as np 
import matplotlib.pyplot as plt
from matplotlib.pyplot import specgram
from scipy.stats import kurtosis, skew
import librosa
import librosa.display 
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from image_recognition_msgs.msg import CategoryProbability, FaceProperties
from miro_msgs.msg import platform_config, platform_sensors, platform_state, platform_mics, \
    platform_control, core_state, core_control, core_config, bridge_config, bridge_stream
from miro_constants import miro


from ros_posenet.msg import Poses, Pose, Keypoint

class audio_display:

    def __init__(self):
        self.i = 0
        self.max_count = 10
        self.display_array_1 = np.zeros(2000 * self.max_count)
        self.display_array_2 = np.zeros(2000 * self.max_count)
        self.count = 0
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(411)
        self.ax.set_ylim([-500,500])
        self.line1, = self.ax.plot(self.display_array_1, color="green", label="left")
        self.ax.legend()
        self.ax2 = self.fig.add_subplot(412)
        self.ax2.set_ylim([-500,500])
        self.line2, = self.ax2.plot(self.display_array_2, color="red", label="right")
        self.ax2.legend()
        self.ax3 = self.fig.add_subplot(413)
        #self.line3, = self.ax3.plot(self.display_array_2, color="blue", label="left-right")
        self.hist = self.ax3.hist(self.display_array_1, bins=100, label="hist left")
        self.ax3.legend()
        self.ax4 = self.fig.add_subplot(414)
        self.specgram = self.ax4.specgram(self.display_array_2, NFFT=1000, Fs=20000)
        self.mics_sub = rospy.Subscriber("/miro/rob01/platform/mics", platform_mics, self.callback_mics)



    def callback_mics(self, data):
        self.display_array_1 = np.roll(self.display_array_1, -2000)
        self.display_array_2 = np.roll(self.display_array_2, -2000)

        self.display_array_1[(self.max_count-1)*2000:] = data.data[::2]
        self.display_array_2[(self.max_count-1)*2000:] = data.data[1::2]
        # self.count = (self.count + 1) % self.max_count
        if self.i == 0:
            print("Last second:")
            hist_mat = np.ones(shape=(10,3))
            for i in range(10):
                hist = np.histogram(np.abs(self.display_array_1[i*2000:(i+1)*2000]), bins=[0,1,5,10,50,500,2000])[0]
                #print(hist[3:])
                hist_mat[i] = hist[3:]

            large = np.count_nonzero(hist_mat[:,2])
            middle = np.count_nonzero(hist_mat[:,1])
            small = np.count_nonzero(hist_mat[:,0])

            if small == 10 and middle == 10 and large < 2:
                print("Beep!")
            elif 3 <= small <= 5 and 3 <= middle <= 5 and large < 2:
                print("BeepBeep!")
            elif 2 <= middle <= 5 and 2 <= large <= 3:
                print("Clap!")
        self.i = (self.i+1) % 10
     

def main(args):
    ad = audio_display()
    rospy.init_node('audio_display', anonymous=True)
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            rate.sleep()
            ad.line1.set_ydata(ad.display_array_1)
            ad.line2.set_ydata(ad.display_array_2)
            #ad.line3.set_ydata(ad.display_array_1-ad.display_array_2)
            ad.ax3.cla()
            #ad.hist = ad.ax3.hist(ad.display_array_1, bins=100, label="hist left")
            #print(ad.display_array_1.shape)
            #print(np.sum(np.abs(ad.display_array_1)))
            # print("New data")
            # for i in range(10):
            #     print(np.histogram(np.abs(ad.display_array_1[i*20000:(i+1)*20000]), bins=[0,5,10,50,500,2000])[0])

            #ad.ax3.relim()
            #ad.ax3.autoscale_view()

            # ad.specgram = ad.ax4.specgram(ad.display_array_2, Fs=22050)

            #print(ad.display_array)
            time = rospy.Time.now()
            mfccs = librosa.feature.mfcc(y=ad.display_array_1, sr=22050, n_mfcc=20)
            # melspectrogram = librosa.feature.melspectrogram(y=ad.display_array_1, sr=22050)

            # print((rospy.Time.now() - time).to_sec())
            # print(mfccs.shape)
            
            mfccs_min = np.min(mfccs, axis=1)  # row-wise summaries
            mfccs_max = np.max(mfccs, axis=1)
            mfccs_median = np.median(mfccs, axis=1)
            mfccs_mean = np.mean(mfccs, axis=1)
            mfccs_variance = np.var(mfccs, axis=1)
            mfccs_skeweness = skew(mfccs, axis=1)
            mfccs_kurtosis = kurtosis(mfccs, axis=1)

            ad.specgram = librosa.display.specshow(mfccs, x_axis='time')

            onset_frames = librosa.onset.onset_detect(y=ad.display_array_1, sr=22050)
            #print(librosa.frames_to_time(onset_frames, sr=22050))
            onset_env = librosa.onset.onset_strength(y=ad.display_array_1, sr=22050)
            #print(librosa.beat.tempo(onset_envelope=onset_env, sr=22050))

            # print(np.array([mfccs_min, mfccs_max, mfccs_median, mfccs_mean, mfccs_variance, mfccs_skeweness, mfccs_kurtosis]).shape)

            ad.fig.canvas.draw()
            ad.fig.canvas.flush_events()

    except KeyboardInterrupt:
        print("Shutting down")
        #librosa.output.write_wav('test.wav', ad.display_array_1, sr=22050, norm=True)



if __name__ == '__main__':
    main(sys.argv)