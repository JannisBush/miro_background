#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage, Range
from geometry_msgs.msg import Twist

import miro_msgs
from miro_msgs.msg import platform_config,platform_sensors,platform_state,platform_mics,platform_control,core_state,core_control,core_config,bridge_config,bridge_stream

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

class explore:

    def callback_range(self, object):
        
        # ignore until active
        if not self.active:
            return

        # store object
        self.range = object

        # send downstream command, ignoring upstream data
        q = platform_control()

        # timing
        sync_rate = 50
        period = 2 * sync_rate # two seconds per period
        z = self.count / period

        # count
        self.count = self.count + 1
        if self.count == 400:
            self.count = 0

   
        # advance pattern
        if not z == self.z_bak:
            self.z_bak = z
            
            # create body_vel for next pattern segment
            self.body_vel = Twist()
            if self.drive_pattern == "turn":
                # turn 90deg left 
                print "turn left and move backwards"
                self.body_vel.angular.z = +0.7854
                self.body_vel.linear.x = -050
                self.drive_pattern = "explore"
                self.count = 0

            else:
                # do-si-do

                if z == 0:
                    print "turn head right"
                    self.body_config = miro.MIRO_YAW_MIN_RAD
                    self.body_config_speed = miro.MIRO_P2U_W_LEAN_SPEED_INF
                if z == 1:
                    print "turn head left"
                    self.body_config = miro.MIRO_YAW_MAX_RAD
                    self.body_config_speed = miro.MIRO_P2U_W_LEAN_SPEED_INF
                if z == 2:
                    print "turn head straight"
                    self.body_config = 0.5 * (miro.MIRO_YAW_MAX_RAD - miro.MIRO_YAW_MIN_RAD) + miro.MIRO_YAW_MIN_RAD
                    self.body_config_speed = miro.MIRO_P2U_W_LEAN_SPEED_INF
                if z == 3:
                    if self.range.range > 0.20:
                        print "drive forward"
                        self.body_vel.linear.x = +050
                    else:
                        print "obstacle, turn"
                        self.drive_pattern = "turn"

                # no head turning at the moment
                self.body_config = 0.5 * (miro.MIRO_YAW_MAX_RAD - miro.MIRO_YAW_MIN_RAD) + miro.MIRO_YAW_MIN_RAD
                self.body_config_speed = miro.MIRO_P2U_W_LEAN_SPEED_INF


        
		# point cameras down
		#q.body_config[1] = 1.0
		#q.body_config_speed[1] = miro.MIRO_P2U_W_LEAN_SPEED_INF

        # publish
        q.body_vel = self.body_vel
        q.body_config[2] = self.body_config
        q.body_config_speed[2] = self.body_config_speed
        # print q
        self.pub_platform_control.publish(q)
        

    def callback_platform_state(self, object):
        
        # ignore until active
        if not self.active:
            return

        # store object
        self.platform_state = object

        # 12 == #define MIRO_P1_R_DESTALL_ON_UC1  (UC1 board for wheels) means wheels are blocked 

        if self.platform_state.P1_R_signals !=0:
            print self.platform_state.P1_R_signals

        if self.platform_state.P1_R_signals == 12:
            # don't do other action until finished
            print "retreat motors blocked"
            self.active = False
            q = platform_control()
            # move backwards and turn 
            q.body_vel.linear.x = -075
            q.body_vel.angular.z = -0.5
            self.pub_platform_control.publish(q)
            # wait a short amount of time
            time.sleep(2.5)
            # stop
            q.body_vel.linear.x = 0
            q.body_vel.angular.z = 0
            self.pub_platform_control.publish(q)
            # self.drive_pattern = "turn"
            # activaite again
            self.count = 0
            self.active = True

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
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

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
        self.drive_pattern = ""

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
                self.robot_name = val
            elif key == "drive":
                self.drive_pattern = val
            else:
                error("argument not recognised \"" + arg + "\"")

        # check we got at least one
        if len(self.robot_name) == 0:
            error("argument \"robot\" must be specified")
        
        # pattern
        self.count = 0
        self.z_bak = -1
        self.body_vel = None

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
        self.pub_bridge_config = rospy.Publisher(topic_root + "/bridge/config",
                    bridge_config, queue_size=0)
        self.pub_bridge_stream = rospy.Publisher(topic_root + "/bridge/stream",
                    bridge_stream, queue_size=0)
        self.pub_platform_config = rospy.Publisher(topic_root + "/platform/config",
                    platform_config, queue_size=0)

        # subscribe
        self.sub_sensors = rospy.Subscriber("/miro/range",
                Range, self.callback_range)
        self.sub_state = rospy.Subscriber(topic_root + "/platform/state",
                platform_state, self.callback_platform_state)
        self.sub_mics = rospy.Subscriber(topic_root + "/platform/mics",
                platform_mics, self.callback_platform_mics)
        self.sub_core_state = rospy.Subscriber(topic_root + "/core/state",
                core_state, self.callback_core_state)
        
        # set active
        self.active = True

if __name__ == "__main__":
    rospy.init_node("explore", anonymous=True)
    main = explore()
    main.loop()
    


