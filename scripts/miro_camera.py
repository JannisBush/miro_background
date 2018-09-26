#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import yaml

class image_converter:

  def __init__(self, topic_root):

    yaml_fname = rospy.get_param('~pathToYAMLCamCalFile')

    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    
    # Parse
    self.camera_info_msg = CameraInfo()
    self.camera_info_msg.width = calib_data["image_width"]
    self.camera_info_msg.height = calib_data["image_height"]
    self.camera_info_msg.K = calib_data["camera_matrix"]["data"]
    self.camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    self.camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    self.camera_info_msg.P = calib_data["projection_matrix"]["data"]
    self.camera_info_msg.distortion_model = calib_data["distortion_model"]

    self.image_pub = rospy.Publisher("/image", Image, queue_size=1)
    self.camera_info_pub = rospy.Publisher("/camera_info", CameraInfo, queue_size=1)

    self.image_sub = rospy.Subscriber("/camera/left/raw", Image, self.callback)

  def callback(self, image):

    timeNow = rospy.Time.now()
    self.camera_info_msg.header.stamp =  timeNow
    image.header.stamp = timeNow

    self.image_pub.publish(image)
    self.camera_info_pub.publish(self.camera_info_msg)



def main(topic_root):
  print("main")
  enc = image_converter(topic_root)
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