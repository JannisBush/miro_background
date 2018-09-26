#!/usr/bin/env python


################################################################
import time
import sys
import math

import numpy
import rospy


from std_msgs.msg import String, UInt16MultiArray, Int8
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist, Point

from miro_constants import miro
from miro_msgs.msg import platform_config, platform_sensors, platform_state, platform_mics, platform_control, \
    core_state, core_control, core_config, bridge_config, bridge_stream

################################################################

def fmt(x, f):
    s = ""
    x = bytearray(x)
    for i in range(0, len(x)):
        if not i == 0:
            s = s + ", "
        s = s + f.format(x[i])
    return s


def hex2(x):
    return "{0:#04x}".format(x)


def hex4(x):
    return "{0:#06x}".format(x)


def hex8(x):
    return "{0:#010x}".format(x)


def flt3(x):
    return "{0:.3f}".format(x)


def error(msg):
    print(msg)
    sys.exit(0)


def usage():
    print("""
Usage:
    miro_teleop.py robot=<robot_name>

    Without arguments, this help page is displayed. To run the
    client you must specify at least the option "robot".

Options:
    robot=<robot_name>
        specify the name of the miro robot to connect to,
        which forms the ros base topic "/miro/<robot_name>".
        there is no default, this argument must be specified.
    """)
    sys.exit(0)


################################################################

class miro_teleop:

    def check_safe(self, safe_int):
        if safe_int.data == 0: 
            active = True
        else:
            active = False

        if active != self.active:
            self.active = active

    def callback_teleop(self, twist):

        # ignore until active
        if not self.active:
            return

        # self.msg = platform_control()

        twist.linear.x = 200 * twist.linear.x

        twist.linear.y = 200 * twist.linear.y  # no effect on miro
        twist.linear.z = 200 * twist.linear.z  # no effect on miro

        self.msg.body_vel = twist

        self.change()


    def callback_head(self, head_data):
      
        # ignore until active
        if not self.active:
            return

        # print(head_data)

        # self.msg = platform_control()

        x = head_data.x * 0.01
        self.msg.body_config[1] = x * (miro.MIRO_LIFT_MAX_RAD - miro.MIRO_LIFT_MIN_RAD) + miro.MIRO_LIFT_MIN_RAD
        self.msg.body_config_speed[1] = miro.MIRO_P2U_W_LEAN_SPEED_INF
       
        x = head_data.y * 0.01
        self.msg.body_config[2] = x * (miro.MIRO_YAW_MAX_RAD - miro.MIRO_YAW_MIN_RAD) + miro.MIRO_YAW_MIN_RAD
        self.msg.body_config_speed[2] = miro.MIRO_P2U_W_LEAN_SPEED_INF
        
        x =  head_data.z * 0.01
        self.msg.body_config[3] = x * (miro.MIRO_PITCH_MAX_RAD - miro.MIRO_PITCH_MIN_RAD) + miro.MIRO_PITCH_MIN_RAD
        self.msg.body_config_speed[3] = miro.MIRO_P2U_W_LEAN_SPEED_INF

        self.change()

    def change(self):
        # ignore until active
        if not self.active:
            return
            
        self.pub_platform_control.publish(self.msg) #Publish to MiRo


    def loop(self):
        rate = rospy.Rate(20)  # 20hz
        self.active = True
        #self.msg = platform_control()
        while not rospy.is_shutdown():

            rate.sleep()
            #self.pub_platform_control.publish(self.msg) #Publish to MiRo


            
    def __init__(self, topic_root):

        # report
        print("initialising...")
        print(sys.version)

        # set inactive
        self.active = False

        self.msg = platform_control()

        # publish
        self.pub_platform_control = rospy.Publisher(topic_root + "/platform/control",
                                                    platform_control, queue_size=1)

        # subscribe
        self.sub_teleop = rospy.Subscriber("cmd_vel", Twist, self.callback_teleop)

        self.sub_teleop = rospy.Subscriber("head_pos", Point, self.callback_head)

        self.sub_safe = rospy.Subscriber(topic_root + "/safe", Int8, self.check_safe)

if __name__ == "__main__":
    # no arguments gives usage
    if len(sys.argv) == 1:
        usage()

    # options
    robot_name = ""
    name = "miro_teleop_converter"

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
    main = miro_teleop(topic_root)
    main.loop()
