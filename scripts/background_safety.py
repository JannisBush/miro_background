#!/usr/bin/env python


################################################################

import rospy
from std_msgs.msg import String, UInt16MultiArray, Int8
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist, Point

import miro_msgs
from miro_msgs.msg import platform_config, platform_sensors, platform_state, platform_mics, platform_control, \
    core_state, core_control, core_config, bridge_config, bridge_stream

import math
import numpy
import time
import sys
from miro_constants import miro


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

class miro_background:

    def callback_sensors(self, data):

        if not self.active:
            return

        self.sensors = data

        # check dangerous situations
        state = 0

        # sonar gives back a sginal between 0.00 (exclusive) and 0.05 (near obstacle)
        if 0.00 < self.sensors.sonar_range.range < 0.05:
            state = 1

        # cliff give back a signal that no surface was detected
        if self.sensors.cliff[0] < 2 or self.sensors.cliff[1] < 2:
            state = 3


        # change state if dangerous situation was found
        if (self.state != state and state != 0):
            self.state = state
            # clear the danger
            #self.clear_danger()

    def callback_state(self, data):

        if not self.active:
            return

        self.platform_state = data

        # check dangerous situations
        state = 0

        # P1_R_signals == 12 (joint stalled)

        if self.platform_state.P1_R_signals == 12:
            state = 3

        # change state if dangerous situation was found
        if (self.state != state and state != 0):
            self.state = state
            # clear the danger
            #self.clear_danger()

    def callback_control(self, data):

        if not self.active:
            return

        if not self.state == 0:
            return

        if self.sensors is None:
            return

        self.control = data


        # save the last actions 
        # for body
        if self.control.body_vel.linear.x is not 0:
            self.last_body_move = "forward" if self.control.body_vel.linear.x > 0 else "backward"

        if self.control.body_vel.angular.z is not 0:
            self.last_body_turn = "right" if self.control.body_vel.angular.z > 0 else "left"

        # for head
        if self.sensors.joint_state.position[1] is not self.control.body_config[1]:
            self.last_head_change[1] = self.control.body_config[1] - self.sensors.joint_state.position[1]

        if self.sensors.joint_state.position[2] is not self.control.body_config[2]:
            self.last_head_change[2] = self.control.body_config[2] - self.sensors.joint_state.position[2]

        if self.sensors.joint_state.position[3] is not self.control.body_config[3]:
            self.last_head_change[3] = self.control.body_config[3] - self.sensors.joint_state.position[3]       

 
    def callback_mics(self, data):

        if not self.active:
            return

        self.mics = data

        # check dangerous situations
        state = 0

        # a really loud situation is dangerous?

        # change state if dangerous situation was found
        if (self.state != state and state != 0):
            self.state = state
            # clear the danger
            #self.clear_danger()

    def callback_camr(self, data):

        if not self.active:
            return

        self.camr = data

        # check dangerous situations
        state = 0

        # a really dark situation is dangerous?

        # dangerous obstacles are near?

        # change state if dangerous situation was found
        if (self.state != state and state != 0):
            self.state = state
            # clear the danger
            #self.clear_danger()

    def callback_caml(self, data):

        if not self.active:
            return

        self.caml = data

        # check dangerous situations
        state = 0

        # a really dark situation is dangerous?

        # dangerous obstacles are near?

        # change state if dangerous situation was found
        if (self.state != state and state != 0):
            self.state = state
            # clear the danger
            #self.clear_danger()


    def clear_danger(self, state):

        if state == 0:
            return

        print(self.state )
        print(self.last_body_move)
        print(self.last_body_turn)
        print(self.last_head_change)

        # state 0= safe, 1 = move backwards, 2 = move forward, 3 = unclear/do nothing/ or turn etc.
        q = platform_control()

        # clear danger depending on memory of last actions 

        # find out what is/what happening using current sensor information and last_head_move, last_body_move etc.

        # default wait

        # move backwards
        if self.last_body_move == "forward" and state == 1:
            q.body_vel.linear.x = -100

        # move forward 
        elif self.last_body_move == "backward" and self.platform_state.P1_R_signals == 12:
            q.body_vel.linear.x = +100

        # turn right

        # turn left 

        self.pub_platform_control.publish(q)

        # check if danger is still there, if yes repeat, else set state to safe
        if self.sensors.sonar_range.range > 0.05 and self.platform_state.P1_R_signals != 12 and self.sensors.cliff[0] > 2 and self.sensors.cliff[1] > 2:
            self.state = 0




    def loop(self):
        rate = rospy.Rate(10)  # 10hz
        self.active = True
        #self.msg = platform_control()
        while not rospy.is_shutdown():
            self.pub_safe.publish(self.state)
            rate.sleep()
            #self.pub_platform_control.publish(self.msg) #Publish to MiRo


            
    def __init__(self, topic_root):

        # report
        print("initialising...")
        print(sys.version)

        # set inactive
        self.active = False
        self.state = 0
        self.last_head_change = [0,0,0,0]
        self.last_body_move = 0
        self.last_body_turn = 0
        self.sensors = None

        # publish
        self.pub_safe = rospy.Publisher(topic_root + "/safe",
                                                    Int8, queue_size=0)

        self.pub_platform_control = rospy.Publisher(topic_root + "/platform/control", platform_control, queue_size=1)

        # subscribe
        self.sub_platform_sensors = rospy.Subscriber(topic_root + "/platform/sensors", platform_sensors, self.callback_sensors)
        self.sub_platform_control = rospy.Subscriber(topic_root + "/platform/control", platform_control, self.callback_control)
        self.sub_platform_state = rospy.Subscriber(topic_root + "/platform/state", platform_state, self.callback_state)

        self.sub_platform_camr = rospy.Subscriber(topic_root + "/platform/camr", Image, self.callback_camr)
        self.sub_platform_caml = rospy.Subscriber(topic_root + "/platform/caml", Image, self.callback_caml)

        self.sub_platforrm_mics = rospy.Subscriber(topic_root + "/platform/mics", platform_mics, self.callback_mics)

        self.sub_safe = rospy.Subscriber(topic_root + "/safe", Int8, self.clear_danger)

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
    main = miro_background(topic_root)
    main.loop()
