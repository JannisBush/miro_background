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
    recorded_sounds_random_lights.py robot=<robot_name>

    Without arguments, this help page is displayed. To run the
    client you must specify at least the option "robot".

Options:
    robot=<robot_name>
        specify the name of the miro robot to connect to,
        which forms the ros base topic "/miro/<robot_name>".
        there is no default, this argument must be specified.
    """)
    sys.exit(0)

class recorded_sounds_random_lights:

    def __init__(self, topic_root):
        # test direct light control
        self.light_pub = rospy.Publisher(topic_root + "/control/lights", UInt16MultiArray, queue_size=1)
        # send sound commands
        self.pub_platform_control = rospy.Publisher(topic_root + "/platform/control", platform_control, queue_size=1)
        self.pub_bridge_stream = rospy.Publisher(topic_root + "/bridge/stream", bridge_stream, queue_size=1)
        # Receive start signal
        self.sound_sub_P1 = rospy.Subscriber("/soundP1", Int8, self.callback_sound_P1)
        self.sound_sub_P2 = rospy.Subscriber("/soundP2", Int8, self.callback_sound_P2)
        self.sound_sub_P3 = rospy.Subscriber("/soundP3", Int8, self.callback_sound_P3)
        self.light_sub = rospy.Subscriber("/lights", String, self.callback_lights)

    def callback_sound_P1(self, sound_play):
        q = platform_control()
        for i in range(60):
            print("P1:", i)
            q.sound_index_P1 = i
            self.pub_platform_control.publish(q)
            time.sleep(2)

    def callback_sound_P2(self, sound_play):
        q = platform_control()
        for i in range(60):
            print("P2:", i)
            q.sound_index_P2 = i
            self.pub_platform_control.publish(q)
            time.sleep(2)

    def callback_sound_P3(self, sound_number):
        q = bridge_stream()
        q.sound_index_P3 = sound_number.data
        print("P3:", sound_number.data)
        self.pub_bridge_stream.publish(q)

    def callback_lights(self, light_play):
        x = UInt16MultiArray()
        x.data = np.random.randint(0, 256, size=18)
        self.light_pub.publish(x)


def main(topic_root):
    print("main")
    enc = recorded_sounds_random_lights(topic_root)
    rate = rospy.Rate(20)
    try:
        while not rospy.is_shutdown():
            # If uncommented sounds in P1 and P2 won't play anymore! bug
            # x = UInt16MultiArray()
            # x.data = np.random.randint(0, 256, size=18)
            # enc.light_pub.publish(x)
            rate.sleep()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
        # no arguments gives usage
        if len(sys.argv) == 1:
                usage()

        # options
        robot_name = ""
        name = "recorded_sounds_random_lights"

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