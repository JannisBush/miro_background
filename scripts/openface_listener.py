#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String, Int8, UInt16MultiArray
from miro_background.msg import Face, Faces, ActionUnit
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import miro_msgs
from miro_msgs.msg import platform_config, platform_sensors, platform_state, platform_mics, platform_control, \
        core_state, core_control, core_config, bridge_config, bridge_stream
from miro_constants import miro
import time
import numpy as np

class openface_listener:

    def __init__(self, topic_root):
        # test of other light control
        self.light_pub = rospy.Publisher(topic_root + "/control/lights", UInt16MultiArray, queue_size=1)

        self.pub_platform_control = rospy.Publisher(topic_root + "/platform/control", platform_control, queue_size=1)

        self.sub_platform_sensors = rospy.Subscriber(topic_root + "/platform/sensors", platform_sensors, self.callback_sensors)
        self.game = False

        self.face_sub = rospy.Subscriber("/faces", Faces, self.callback)

        self.sound_sub = rospy.Subscriber("/sound", String, self.callback_sound)
        self.index = 0


        # listen to microphones to follow a sound 
        # self.mics_sub = rospy.Subscriber(topic_root + "/platform/mics", platform_mics, self.callback_mics_test)
        self.mics_sub = rospy.Subscriber(topic_root + "/platform/mics", platform_mics, self.callback_mics_follow, queue_size=1)
     
        # mics buffers
        self.w_mics = 320
        self.h_mics = 240
        self.ima = bytearray(self.w_mics*self.h_mics*3)
        self.imb = bytearray(self.w_mics*self.h_mics*3)
        self.spatial_record = 0

        # activate spatial behavior
        self.core_config_pub = rospy.Publisher(topic_root + "/core/config", core_config, queue_size=0)
        time.sleep(3)
        c = core_config()
        c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_ENABLE
        c.msg_flags = c.FLAG_UPDATE_SIGNALS
        self.core_config_pub.publish(c)
        # does not work? ... workaround use the gui to enable it

        # listen to it (core audio event)
        # self.core_sub = rospy.Subscriber(topic_root + "/core/state", core_state, self.callback_core_follow)



    def callback_core_follow(self, core_state):
        if core_state.audio_event.magnitude > 0.1:
            angular = core_state.audio_event.azim
            linear = core_state.audio_event.magnitude / 10
            self.update_body_vel(linear, angular)

    def update_body_vel(self, linear, angular):
        q = platform_control()
        q.body_vel.linear.x = linear * 1000
        q.body_vel.angular.z = angular
        self.pub_platform_control.publish(q)

    def callback_mics_follow(self, mics):

        linear=0
        angular=0

        left_mic = mics.data[::2]
        right_mic = mics.data[1::2]

        corr = np.correlate(left_mic,right_mic,"same")
        max_corr = np.max(corr[994:1006])
        max_index = np.argwhere(corr == max_corr)[0][0]
        # print(max_index, max_corr)

        #if sound is sufficiently loud, move towards sound source
        if max_corr > 400000:
        
            if max_index >= 994 and max_index <= 995:
                linear = .1
                angular = 1
                self.update_body_vel(linear, angular)
                time.sleep(1.4)
            elif max_index >= 996 and max_index <= 997:
                linear = .1
                angular = 1
                self.update_body_vel(linear, angular)
                time.sleep(1)
            elif max_index >= 998 and max_index <= 999:
                linear = .1
                angular = 1
                self.update_body_vel(linear, angular)
                time.sleep(.6)
            elif max_index == 1000:
                linear = .1
                angular = 0
                self.update_body_vel(linear, angular)
            elif max_index >= 1001 and max_index <= 1002:
                linear = .1
                angular = -1
                self.update_body_vel(linear, angular)
                time.sleep(.6)
            elif max_index >= 1003 and max_index <= 1004:
                linear = .1
                angular = -1
                self.update_body_vel(linear, angular)
                time.sleep(1)
            elif max_index >= 1005 and max_index <= 1006:
                linear = .1
                angular = -1
                self.update_body_vel(linear, angular)
                time.sleep(1.4)
            
            angular = 0
            self.update_body_vel(linear, angular)

    def callback_mics_test(self, mics):
        count_in = 0
        self.platform_mics = mics
        # mics
        if not self.platform_mics is None:
                q = self.platform_mics
                self.platform_mics = None
                data = q.data
                w_mics = self.w_mics
                h_mics = self.h_mics
                p1 = h_mics / 4
                p2 = p1 * 3
                o = p1 - 1
                s = w_mics * 3
                for j in range(0, w_mics*h_mics*3):
                        self.ima[j] = 0
                        #self.imb[j] = 0
                for j in range(0, w_mics):
                        i = j * 2000 / w_mics
                        a = data[i*2+0]
                        b = data[i*2+1]
                        print("a1",a)
                        a = a * o / 512
                        print("a2",a)
                        b = b * o / 512
                        x = i * w_mics / 2000
                        self.ima[x*3+(p1-a)*s] = 255
                        self.ima[x*3+(p2-b)*s+1] = 255
                        count_in = count_in + 1
                print(type(self.ima))
                print(len(self.ima))
                print(type(data))
                print(data)
                print(len(data))
                print(count_in)
                print(np.sum(data))
                print(np.sum(self.ima))
                #self.pba = GdkPixbuf.Pixbuf.new_from_data(self.ima,
                #    GdkPixbuf.Colorspace.RGB, False, 8, w_mics, h_mics,
                #    w_mics*3, None, None)
                #self.image_caml.set_from_pixbuf(self.pba)
                #self.pbb = GdkPixbuf.Pixbuf.new_from_data(self.imb,
                #    GdkPixbuf.Colorspace.RGB, False, 8, w_mics, h_mics,
                #    w_mics*3, None, None)
                #self.image_camr.set_from_pixbuf(self.pba)

    def callback_sensors(self, sensors):
        # any head touch is recognized as a game starting point
        if np.sum(np.array(bytearray(sensors.touch_head)).astype(np.float)) > 0:
            # start blink game
            self.start_time = rospy.Time.now()
            self.game = True
            # announce game start by beep
            q = platform_control()
            q.sound_index_P1 = 1
            self.pub_platform_control.publish(q)

    def callback_sound(self, sound_play):
        print("sound")
        # x = UInt16MultiArray()
        # x.data = np.random.randint(0, 256, size=18)
        # self.light_pub.publish(x)
        # return

        q = platform_control()

        # for i in xrange(60):
        #     print(i)
        #     q.sound_index_P1 = i
        #     # q.sound_index_P2 = self.index
        #     self.pub_platform_control.publish(q)
        #     self.index += 1
        #     time.sleep(2)

        q.sound_index_P1 = 0
        for i in xrange(60):
            print(i)
            q.sound_index_P2 = i
            self.pub_platform_control.publish(q)
            time.sleep(2)


    def callback(self, faces):
        if len(faces.faces) != 0:
            q = platform_control()
            
            # prepare lights 
            q.lights_dphase = 64
            q.lights_amp = 128
            q.lights_max_drive = 128

            # prepare aus
            au1 = au2 = au4 = au5 = au6 = au7 = au9 = au12 = au14 = au15 = au20 = au23 = au26 = au45 = False 

            face_actions = faces.faces[0].action_units

            for au in face_actions:

                # print(au.name)
                # print(au.presence)

                # facial action gross behavior code 45: blink
                if au.name == "AU45" and au.presence == 1.0:
                    print("blink detected" )
                    # react by also blinking
                    q.blink_time = 10.0
                    au45 = True

                    # blinking game
                    if self.game == True:
                        time = (rospy.Time.now() - self.start_time).to_sec()
                        print("your blinking game score: " + str(time))
                        # end the game and signalize with beep
                        self.game = False
                        q.sound_index_P1 = 1
                        # if time is high, be happy -> green, else angry = red
                        if time > 30:
                            q.lights_rgb = [0, 255, 0]
                        else:
                            q.lights_rgb = [255, 0, 0]
                        # break the loop
                        break

                # inner brow raiser 
                if au.name == "AU01" and au.intensity > 2.0:
                    print("inner brow raiser detected")
                    au1 = True

                # outer brow raiser 
                if au.name == "AU02" and au.intensity > 2.0:
                    print("outer brow raiser detected")
                    au2 = True

                # brow lowerer 
                if au.name == "AU04" and au.presence == 1.0:
                    print("brow lowerer detected")
                    au4 = True

                # upper lid raiser
                if au.name == "AU05" and au.intensity > 2.0:
                    print("upper lid raiser detected")
                    au5 = True

                # cheeck raiser
                if au.name == "AU06" and au.presence == 1.0:
                    print("cheek raiser detected")
                    au6 = True

                # lid tightener
                if au.name == "AU07" and au.presence == 1.0:
                    print("lid tightener detected")
                    au7 = True

                 # nose wrinkler 
                if au.name == "AU09" and au.presence == 1.0:
                    print("nose wrinkler detected")
                    au9 = True
            
                # lip corner puller 
                if au.name == "AU12" and au.presence == 1.0:
                    print("lip corner puller detected")
                    au12 = True

                # dimpler
                if au.name == "AU14" and au.presence == 1.0:
                    print("dimpler detected")
                    au14 = True
                                    
                # lip corner depressor
                if au.name == "AU15" and au.intensity > 2.0:
                    print("lip corner depressor detected")
                    au15 = True

                # lip stretcher 
                if au.name == "AU20" and au.presence == 1.0:
                    print("lip stretcher detected")
                    au20 = True

                # lip tigthener 
                if au.name == "AU23" and au.presence == 1.0:
                    print("lip tightener detected")
                    au23 = True

                # jaw drop 
                if au.name == "AU26" and au.intensity > 2.0:
                    print("jaw drop detected")
                    au26 = True

                # place for other action units

            # detect emotions (combinations of facial action units)

            # Happiness 6 + 12
            if au6 is True and au12 is True:
                print("happiness detected")
                # react with green light
                q.lights_rgb = [0, 255, 0] #r, g, b

            # Sadness 1 + 4 + 15
            if au1 is True and au4 is True and au15 is True:
                print("sadness detected")
                # react with inappropiate comment
                q.sound_index_P2 = 23

            # Surprise 1 + 2 + 5 + 26 
            if au1 is True and au2 is True and au5 is True and au26 is True:
                print("surprise detected")
                # react with strange sound
                #q.sound_index_P1 = 16
                # react with laugther
                q.sound_index_P2 = 12

            # Fear 1 + 2 + 4 + 5 + 7 + 20 + 26
            if au1 is True and au2 is True and au4 is True and au5 is True and au7 is True and au20 is True and au26 is True:
                print("fear detected")
                # react with blue light
                q.lights_rgb = [0, 0, 255]

            # Anger 4 + 5 + 7 + 23
            if au4 is True and au5 is True and au7 is True and au23 is True:
                print("anger detected")
                # react with red light
                q.lights_rgb = [255, 0, 0]

            # Disgust 9 + 15 (+16 not recongized, therefore ignored)
            if au9 is True and au15 is True:
                print("disgust detected")
                # react with rotating ears
                q.ear_rotate[0] = 1.0

            # Contempt 12 + 14
            if au12 is True and au14 is True:
                print("contempt detected")
                # react with rotating ears
                q.ear_rotate[1] = 1.0

            # place for more emotions etc.

            self.pub_platform_control.publish(q)

def main(topic_root):
    print("main")
    enc = openface_listener(topic_root)
    rate = rospy.Rate(20)
    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
        # no arguments gives usage
        if len(sys.argv) == 1:
                usage()

        # options
        robot_name = ""

        print(sys.argv)

        # handle args
        for arg in sys.argv[1:]:
                f = arg.find('=')
                if f == -1:
                        key = arg
                        val = ""
                else:
                        key = arg[:f]
                        val = arg[f+1:]
                if key == "robot":
                        robot_name = val
                elif key == "__name:":
                        name = val
                elif key == "__log:":
                        pass
                else:
                        print(key)
                        error("argument not recognised \"" + arg + "\"")

        # check we got at least one
        if len(robot_name) == 0:
                error("argument \"robot\" must be specified")

        # topic root
        topic_root = "/miro/" + robot_name
        print("topic_root", topic_root)

        rospy.init_node(name, anonymous=True)
        main(topic_root)