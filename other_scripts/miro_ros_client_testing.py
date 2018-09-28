#!/usr/bin/env python


################################################################

import rospy
from std_msgs.msg import String, UInt16MultiArray
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist

import miro_msgs
from miro_msgs.msg import platform_config, platform_sensors, platform_state, platform_mics, platform_control, \
    core_state, core_control, core_config, bridge_config, bridge_stream

import math
import numpy as np
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

def to_float_array(x):
    return np.array(bytearray(x)).astype(np.float)


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
    print """
Usage:
    miro_ros_client.py robot=<robot_name>

    Without arguments, this help page is displayed. To run the
    client you must specify at least the option "robot".

Options:
    robot=<robot_name>
        specify the name of the miro robot to connect to,
        which forms the ros base topic "/miro/<robot_name>".
        there is no default, this argument must be specified.
    """
    sys.exit(0)


################################################################

class miro_ros_client_testing:

    def callback_turn_180(self, object):

        # ignore until active
        if not self.active:
            return

        # if turn is false, do nothing
        if not self.turn:
            return

        # send downstream command, ignore upstream data
        q = platform_control()

        # timing
        sync_rate = 50
        period = 2 * sync_rate  # two seconds per period
        z = self.count / period

        # advance pattern
        if not z == self.z_bak:
            self.z_bak = z
            # create body_vel for next pattern segment
            self.body_vel = Twist()
            if z < 2:
                print "turn left"
                self.body_vel.angular.z = +0.7854
            if z == 2:
                print "turned"
                self.turn = False
                self.count = 0

        # publish
        q.body_vel = self.body_vel
        self.pub_platform_control.publish(q)

        # count
        self.count = self.count + 1

    def callback_move_until_obstacle(self, object):

        # ignore until active
        if not self.active:
            return

        # if turn is true, do nothing
        if self.turn:
            return

        # store object
        self.platform_sensors = object

        # send downstream command, ignore upstream data
        q = platform_control()

        # create body_vel for next pattern segment
        self.body_vel = Twist()

        if self.platform_sensors.sonar_range.range > 0.20:
            self.body_vel.linear.x = +200
        else:
            print "Obstacle stop"
            self.turn = True

        # publish
        q.body_vel = self.body_vel
        self.pub_platform_control.publish(q)

    def callback_platform_sensors(self, object):

        # ignore until active
        if not self.active:
            return

        # store object
        self.platform_sensors = object

        # print self.platform_sensors
      
        q = core_control()

        if np.sum(to_float_array(self.platform_sensors.light)) > 200:
            q.sleep_drive_target.wakefulness = 0.0
            q.sleep_drive_target.pressure = 0.0
        else:
            q.sleep_drive_target.wakefulness = 1.0
            q.sleep_drive_target.pressure = 1.0

        q.sleep_drive_gamma = 0.5
        # print q
        self.pub_platform_control.publish(platform_control())
        self.pub_core_control.publish(q)

        # rospy.signal_shutdown("because")

    def callback_platform_state(self, object):

        # ignore until active
        if not self.active:
            return

        # store object
        self.platform_state = object

    def callback_platform_mics(self, object):

        # ignore until active
        if not self.active:
            return

        # store object
        self.platform_mics = object

    def callback_core_state(self, object):

        # ignore until active
        if not self.active:
            return

        # store object
        self.core_state = object

    def loop(self):
        rate = rospy.Rate(1) #1hz
        while not rospy.is_shutdown():
            rate.sleep()
        
        if self.test:
            c = core_config()
            c.msg_flags = c.FLAG_UPDATE_SIGNALS
            print c
            self.pub_core_config.publish(c)

    def __init__(self):

        # report
        print("initialising...")
        print(sys.version)

        # default data
        self.platform_sensors = None
        self.platform_state = None
        self.platform_mics = None
        self.core_state = None

        # no arguments gives usage
        if len(sys.argv) == 1:
            usage()

        # options
        self.robot_name = ""
        self.test = False

        # handle args
        for arg in sys.argv[1:]:
            f = arg.find('=')
            if f == -1:
                key = arg
                val = ""
            else:
                key = arg[:f]
                val = arg[f + 1:]
            if key == "robot":
                self.robot_name = val
            elif key == "test":
                self.test = True if val == "True" else False
            else:
                error("argument not recognised \"" + arg + "\"")

        # check we got at least one
        if len(self.robot_name) == 0:
            error("argument \"robot\" must be specified")

        # pattern
        self.count = 0
        self.z_bak = -1
        self.body_vel = None
        self.turn = False

        # set inactive
        self.active = False

        # topic root
        topic_root = "/miro/" + self.robot_name
        print "topic_root", topic_root

        # publish
        self.pub_platform_control = rospy.Publisher(topic_root + "/platform/control",
                                                    platform_control, queue_size=0)
        self.pub_core_control = rospy.Publisher(topic_root + "/core/control",
                                                 core_control, queue_size=0)
        self.pub_core_config = rospy.Publisher(topic_root + "/core/config",
                                                core_config, queue_size=0)
        # self.pub_bridge_config = rospy.Publisher(topic_root + "/bridge/config",
        #                                          bridge_config, queue_size=0)
        # self.pub_bridge_stream = rospy.Publisher(topic_root + "/bridge/stream",
        #                                          bridge_stream, queue_size=0)
        # self.pub_platform_config = rospy.Publisher(topic_root + "/platform/config",
        #                                            platform_config, queue_size=0)

        # subscribe
        if self.test:
            rospy.Subscriber(topic_root + "/platform/sensors",
                                            platform_sensors, self.callback_platform_sensors)
            c = core_config()
            c.P2B_W_signals = c.P2B_W_signals | miro.MIRO_P2B_W_BRANCH_ENABLE
            c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_ENABLE
            c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_DO_PIRATE_NOISES
            c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_ENABLE
            c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_ENABLE_SLEEP

           
            #c.P2U_W_action_signals = c.P2U_W_action_signals | miro.MIRO_P2U_W_ACTION_ENABLE
            #c.P2U_W_action_signals = c.P2U_W_action_signals | miro.MIRO_P2U_W_ACTION_FORCE_MULL
            #c.P2U_W_action_signals = c.P2U_W_action_signals | miro.MIRO_P2U_W_ACTION_RANDOMIZE_ORIENT

            #c.P1_W_signals = c.P1_W_signals | miro.MIRO_P1_W_TEST_ALARM

            c.msg_flags = c.FLAG_UPDATE_SIGNALS
            print c
            self.pub_core_config.publish(c)
            time.sleep(1)
        else:
            self.sub_sensors = rospy.Subscriber(topic_root + "/platform/sensors",
                                                platform_sensors, self.callback_move_until_obstacle)
            rospy.Subscriber(topic_root + "/platform/state",
                                                platform_state, self.callback_turn_180)            
        # self.sub_state = rospy.Subscriber(topic_root + "/platform/state",
        #                                   platform_state, self.callback_platform_state)
        # self.sub_mics = rospy.Subscriber(topic_root + "/platform/mics",
        #                                  platform_mics, self.callback_platform_mics)
        # self.sub_core_state = rospy.Subscriber(topic_root + "/core/state",
        #                                        core_state, self.callback_core_state)

        # set active
        self.active = True

if __name__ == "__main__":
    rospy.init_node("miro_ros_client_py", anonymous=True)
    main = miro_ros_client_testing()
    main.loop()
