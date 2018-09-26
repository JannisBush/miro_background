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

def usage():
    print("""
Usage:
    sound_follow_1_2.py robot=<robot_name>

    Without arguments, this help page is displayed. To run the
    client you must specify at least the option "robot".

Options:
    robot=<robot_name>
        specify the name of the miro robot to connect to,
        which forms the ros base topic "/miro/<robot_name>".
        there is no default, this argument must be specified.
    """)
    sys.exit(0)


class sound_follower:

    def __init__(self, topic_root, follow_type):

        self.pub_platform_control = rospy.Publisher(topic_root + "/platform/control", platform_control, queue_size=1)

        if follow_type == "mics":
            # listen to microphones to follow a sound 
            self.mics_sub = rospy.Subscriber(topic_root + "/platform/mics", platform_mics, self.callback_mics_follow, queue_size=1)
        elif follow_type == "core":
            # activate spatial behavior
            self.core_config_pub = rospy.Publisher(topic_root + "/core/config", core_config, queue_size=0)
            # some publisher that does not hurt
            self.core_control_pub = rospy.Publisher(topic_root + "/core/control", core_control, queue_size=1)
            time.sleep(1)
            self.c = core_config()
            self.c.P2S_W_signals = self.c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_ENABLE
            self.c.msg_flags = self.c.FLAG_UPDATE_SIGNALS
            self.core_config_pub.publish(self.c)
            # listen to it (core audio event)
            self.core_sub = rospy.Subscriber(topic_root + "/core/state", core_state, self.callback_core_follow)
        else:
            print("Enter a valid mode: mics|core")
            rospy.signal_shutdown("Enter a valid mode: mics|core")

    def update_body_vel(self, linear, angular):
        q = platform_control()
        q.body_vel.linear.x = linear * 1000
        q.body_vel.angular.z = angular
        self.pub_platform_control.publish(q)

    def callback_core_follow(self, core_state):
        if core_state.audio_event.magnitude > 0.1:
            angular = core_state.audio_event.azim
            linear = core_state.audio_event.magnitude / 10
            self.update_body_vel(linear, angular)

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


def main(topic_root, follow_type):
    print("main")
    enc = sound_follower(topic_root, follow_type)
    rate = rospy.Rate(20)
    try:
        while not rospy.is_shutdown():
            rate.sleep()
            # send some messages, so that the core behavior works
            if follow_type == "core":
                x = core_control()
                enc.core_control_pub.publish(x)
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
        # no arguments gives usage
        if len(sys.argv) == 1:
                usage()

        # options
        robot_name = ""
        follow_type = "mics"
        name = "sound_follower"

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
                elif key == "follow_type":
                    follow_type = val
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
        print("follow_type", follow_type)

        rospy.init_node(name, anonymous=True)
        main(topic_root, follow_type)