#!/usr/bin/env python

import roslib
import sys
import rospy
from std_msgs.msg import String, Int8
from sensor_msgs.msg import CompressedImage, Range, Image, LaserScan
from nav_msgs.msg import Odometry
import miro_msgs
from miro_msgs.msg import platform_config, platform_sensors, platform_state, platform_mics, platform_control, \
    core_state, core_control, core_config, bridge_config, bridge_stream
import math
import tf
import geometry_msgs.msg 
import tf2_ros
import tf2_msgs.msg
import numpy as np

def usage():
    print("""
Usage:
    sensor_to_pos.py robot=<robot_name>

    Without arguments, this help page is displayed. To run the
    client you must specify at least the option "robot".

Options:
    robot=<robot_name>
        specify the name of the miro robot to connect to,
        which forms the ros base topic "/miro/<robot_name>".
        there is no default, this argument must be specified.
    """)
    sys.exit(0)

class sensor_to_pos:

  def __init__(self, topic_root):
    print("init")
    self.sensors = None

    # array to hold the last 10 range measurements (at the start a high value)
    self.range_data = np.ones(10)

    self.range_pub = rospy.Publisher("/range", Range, queue_size=10)
    self.laser_pub = rospy.Publisher("/scan", LaserScan, queue_size=10)
    self.odom_pub = rospy.Publisher("/miro/odom", Odometry, queue_size=1)
    self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

    self.sensor_sub = rospy.Subscriber(topic_root + "/platform/sensors", platform_sensors, self.callback_sensor)


    # self.image_sub = rospy.Subscriber(topic_root + "/platform/caml",Image,self.callback_im)
    # self.image_pub = rospy.Publisher("/miro/camera/image",Image,queue_size=1)

  def callback_im(self, image):
    self.image_pub.publish(image)

  def callback_sensor(self, data):

    print("callback_sensor")

    # shift all elements in the range data by one and replace the first item 
    self.range_data = np.roll(self.range_data, 1)
    self.range_data[0] = data.sonar_range.range
    
    if self.sensors is None:
      self.sensors = data
      self.linear_x = 0
      self.angular_z = 0
      self.time_last = self.sensors.odometry.header.stamp

      ##https://www.cs.cmu.edu/afs/cs.cmu.edu/academic/class/16311/www/s07/labs/NXTLabs/Lab%203.html##
      self.x = 0
      self.y = 0
      self.theta = 0
      ##

      ##https://github.com/RiccardoGiubilato/ros_autonomous_car##
      self.miro_pos_br = tf.TransformBroadcaster()
      ##

    else:
      self.sensors = data
      # print("x:", self.sensors.accel_body.linear_acceleration.x - data.accel_body.linear_acceleration.x)
      # print("y:", self.sensors.accel_body.linear_acceleration.y - data.accel_body.linear_acceleration.y)
      # print("z:", self.sensors.accel_body.linear_acceleration.z - data.accel_body.linear_acceleration.z)
      time_since_last = (data.odometry.header.stamp - self.time_last).to_sec()
      # print("time since last time:", time_since_last)
      self.time_last = data.odometry.header.stamp

      # self.linear_x += data.odometry.twist.twist.linear.x * time_since_last * 0.1
      # print("linear x (cm):", self.linear_x )

      # self.angular_z += data.odometry.twist.twist.angular.z  * time_since_last
      # print("angular z (rad):", self.angular_z)

      # self.pos = math.cos(self.angular_z)*self.linear_x, math.sin(self.angular_z)*self.linear_x
      # print("pos x,y (cm):", self.pos)

      ##cs.cmu##
      v = data.odometry.twist.twist.linear.x * 0.001
      omega = data.odometry.twist.twist.angular.z
      t = time_since_last

      k00 = v * math.cos(self.theta)
      k01 = v * math.sin(self.theta)
      k02 = omega

      k10 = v * math.cos(self.theta + 0.5 * t * k02)
      k11 = v * math.sin(self.theta + 0.5 * t * k02)
      k12 = omega

      k20 = v * math.cos(self.theta + 0.5 * t * k12)
      k21 = v * math.sin(self.theta + 0.5 * t * k12)
      k22 = omega

      k30 = v * math.cos(self.theta + t * k22)
      k31 = v * math.sin(self.theta + t * k22)
      k32 = omega

      x = self.x + (t/6) * (k00 + 2 * (k10 + k20) + k30)
      y = self.y + (t/6) * (k01 + 2 * (k11 + k21) + k31)
      theta = self.theta + (t/6) * (k02 + 2 * (k12 + k22) + k32)

      # if the change is to large, there was some error in the data, therefore ignore
      if (abs(self.x-x) + abs(self.y-y) > 0.5 or abs(self.theta-theta) > 0.5):
        return
      else:
        self.x = x
        self.y = y
        self.theta = theta

      print("pos x,y,theta (m,m,rad):", self.x, self.y, self.theta)
      ##

      o = Odometry()
      o.pose.pose.position.x = self.x
      o.pose.pose.position.y = self.y
      o.pose.pose.position.z = self.theta

      o.twist = data.odometry.twist
      o.twist.twist.linear.x = o.twist.twist.linear.x / 100

      o.header.frame_id = "odom"
      o.child_frame_id = "base_link"


      self.odom_pub.publish(o)

      timeNow = rospy.Time.now()
      ##github##
      self.miro_pos_br.sendTransform((self.x, self.y, 0), 
        tf.transformations.quaternion_from_euler(0, 0, self.theta),  # makes it more inaccurate?
        # tf.transformations.quaternion_from_euler(0, 0, 0),
        timeNow,
        "base_link",
        "odom")

      self.miro_pos_br.sendTransform((0.15, 0, 0),
        tf.transformations.quaternion_from_euler(0, 0, data.joint_state.position[2]),
        timeNow,
        "sonar_link",
        "base_link")
      ##

      data.sonar_range.header.frame_id = "sonar_link"
      data.sonar_range.header.stamp = timeNow
      data.sonar_range.range = (np.sum(self.range_data) - np.max(self.range_data) - np.min(self.range_data)) / (self.range_data.size -2)
      # publish the mean sonar over the last (20) sonar ranges 
      print(data.sonar_range.range)
      self.range_pub.publish(data.sonar_range)

      Scan_msg = LaserScan()
      Scan_msg.header.frame_id = "sonar_link"
      Scan_msg.range_min = 0.0
      Scan_msg.range_max = 1.00
      Scan_msg.header.stamp = timeNow
      Scan_msg.ranges = [data.sonar_range.range]
      Scan_msg.intensities = [100]
      self.laser_pub.publish(Scan_msg)


      print("")



def main(topic_root, name):
  print("main")
  enc = sensor_to_pos(topic_root)
  rate = rospy.Rate(10)
  try:
    while not rospy.is_shutdown():
      rate.sleep()
      # t = geometry_msgs.msg.TransformStamped()
      # t.header.frame_id = "miro_base_link"
      # t.header.stamp = rospy.Time.now()
      # t.child_frame_id = "sonar_link"
      # t.transform.translation.x = 0.15
      # t.transform.translation.y = 0.0
      # t.transform.translation.z = 0.0

      # t.transform.rotation.x = 0.0
      # t.transform.rotation.y = 0.0
      # t.transform.rotation.z = enc.sensors.joint_state.position[2]
      # t.transform.rotation.w = 1.0

      # tfm = tf2_msgs.msg.TFMessage([t])
      # enc.pub_tf.publish(tfm)

    #rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    # no arguments gives usage
    if len(sys.argv) == 1:
        usage()

    # options
    robot_name = ""
    name = "sensor_to_pos"

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
    main(topic_root, name)