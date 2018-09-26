#!/usr/bin/env python

import sys
import rospy
from miro_background.srv import *
import time

def control_test():
    rospy.wait_for_service('up_body_vel')
    rospy.wait_for_service('stop_move')
    up_body_vel = rospy.ServiceProxy('up_body_vel', BodyVel)
    stop_move = rospy.ServiceProxy('stop_move', StopMove)
    while not rospy.is_shutdown():
        try:
            print("update BodyVel")
            up_body_vel(100, 0.1)
            time.sleep(3)
            stop_move(True)
            print("Stop")
            time.sleep(3)
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)



if __name__ == "__main__":
    control_test()