#!/usr/bin/env python

from miro_background.srv import *
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import Temperature, Imu, JointState, Range
from nav_msgs.msg import Odometry
from miro_msgs.msg import platform_control, platform_sensors
from array import array

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class platform_control_server:

	def update(self):
		# Output new control command
		cmd = platform_control()
		cmd.body_vel = self.body_vel
		cmd.body_config = self.body_config
		cmd.body_config_speed = self.body_config_speed
		cmd.body_move = self.body_move
		cmd.tail = self.tail
		cmd.ear_rotate = self.ear_rotate
		cmd.eyelid_closure = self.eyelid_closure_out
		cmd.blink_time = self.blink_time
		cmd.lights_max_drive = self.lights_max_drive
		cmd.lights_dphase = self.lights_dphase
		cmd.lights_phase = self.lights_phase
		cmd.lights_amp = self.lights_amp
		cmd.lights_off = self.lights_off
		cmd.lights_rgb = self.lights_rgb
		cmd.sound_index_P1 = self.sound_index_P1 
		cmd.sound_index_P2 = self.sound_index_P2
		self.cmd_out.publish(cmd)

	def up_body_vel(self, req):
		self.body_vel.linear.x = req.linear
		self.body_vel.angular.z = req.angular
		return True

	def stop_move(self, req):
		self.body_vel = Twist()
		return True

	def __init__(self, robot_name):
		rospy.init_node('platform_control_server')
		s1 = rospy.Service('up_body_vel', BodyVel, self.up_body_vel)
		s2 = rospy.Service('stop_move', StopMove, self.stop_move)
		# Main publisher / subscriber
		root = "/miro/" + str(robot_name) + "/platform"
		self.cmd_out = rospy.Publisher(root + "/control", platform_control, queue_size=1)
		# Actuators
		self.body_vel = Twist()
		self.body_config = [0.0, 0.0, 0.0, 0.0]
		self.body_config_speed = [0.0, 0.0, 0.0, 0.0]
		self.body_move = Pose2D()
		self.tail = 0.0
		self.ear_rotate = [0.0, 0.0]
		self.eyelid_closure_out = 0.0
		self.blink_time = 0
		self.lights_max_drive = 127
		self.lights_dphase = 0
		self.lights_phase = 0
		self.lights_amp = 0
		self.lights_off = 0
		self.lights_rgb = [0, 0, 0]
		self.sound_index_P1 = 0
		self.sound_index_P2 = 0
		print("Ready to Control MiRo.")
		rate = rospy.Rate(50)
		try:
			while not rospy.is_shutdown():
				self.update()
				rate.sleep()
		except KeyboardInterrupt:
			print("Shutting down")


if __name__ == "__main__":
    platform_control_server("rob01")