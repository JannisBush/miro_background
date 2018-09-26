#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String, Int32MultiArray
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import miro_msgs
from miro_msgs.msg import platform_config, platform_sensors, platform_state, platform_mics, platform_control, \
    core_state, core_control, core_config, bridge_config, bridge_stream
from miro_constants import miro

class face_safety:

  def __init__(self, topic_root):

    self.angle = 90
    self.image_current = None

    self.bridge = CvBridge()

    self.image_pub = rospy.Publisher("/facerec/input_image", Image, queue_size=1)
    self.platform_pub = rospy.Publisher(topic_root + "/platform/control", platform_control, queue_size=1)

    self.image_sub = rospy.Subscriber("/camera/left/raw", Image, self.callback_image)


    self.face_sub = rospy.Subscriber("faceCoord", Int32MultiArray, self.callback_faces)

  def callback_image(self, image):

    self.image_current = image

  def callback_faces(self, face_array):
    # face detected!
    if face_array.data[1] != 0:
        q = platform_control()
        q.lights_dphase = 64
        q.lights_amp = 128
        q.lights_max_drive = 128
        q.lights_rgb = [255, 0, 0]
        q.sound_index_P1 = 5
        self.platform_pub.publish(q)

  def check_faces(self):

    if self.image_current is None:
        return

    try:
      cv_image = self.bridge.imgmsg_to_cv2(self.image_current, "bgr8")
    except CvBridgeError as e:
      print(e)

    # get image height, width
    (h, w) = cv_image.shape[:2]
                        
    # calculate the center of the image
    (cX, cY) = (w // 2, h // 2)
    scale = 1.0
    self.angle = -90 if self.angle == 90 else 90
     
    # Perform the (counter) clockwise rotation holding at the center
    # 90 degrees or -90 degrees
    M = cv2.getRotationMatrix2D((cX, cY), self.angle, scale)
    cos = np.abs(M[0, 0])
    sin = np.abs(M[0, 1])
 
    # compute the new bounding dimensions of the image
    nW = int((h * sin) + (w * cos))
    nH = int((h * cos) + (w * sin))
 
    # adjust the rotation matrix to take into account translation
    M[0, 2] += (nW / 2) - cX
    M[1, 2] += (nH / 2) - cY
    rotated90 = cv2.warpAffine(cv_image, M, (nW, nH)) 

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(rotated90, "bgr8"))  
    except CvBridgeError as e:
      print(e)




def main(topic_root):
  print("main")
  enc = face_safety(topic_root)
  rate = rospy.Rate(1)
  try:
    while not rospy.is_shutdown():
        rate.sleep()
        enc.check_faces()
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