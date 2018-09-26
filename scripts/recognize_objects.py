#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import roslib
import rosservice
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from image_recognition_msgs.msg import CategoryProbability, FaceProperties

from ros_posenet.msg import Poses, Pose, Keypoint
from miro_background.msg import ObjectSide

class toy_recognizer_client:
    """Client that sends images from MiRo's camera stream to the toy_reconizer server."""
    def __init__(self):
        """Prepare the client."""
        # No Images at the start
        self.img_left = None
        self.img_right = None
        # Prepare the bridge and the camera stream subscribers
        self.bridge = CvBridge()
        topic_left = rospy.get_param("topic_left", "/miro/rob01/platform/caml")
        topic_right = rospy.get_param("topic_right", "/miro/rob01/platform/camr")
        self.image_sub_left = rospy.Subscriber(topic_left, Image, self.callback_img_left)
        self.image_sub_right = rospy.Subscriber(topic_right, Image, self.callback_img_right)
        # Wait until the service is ready and connect to it
        rospy.wait_for_service("/recognize")
        self._srv = rospy.ServiceProxy("/recognize", rosservice.get_service_class_by_name("/recognize"))
        # Announce the toy publisher
        self.toy_pub = rospy.Publisher("/toys", ObjectSide, queue_size=0)

    def callback_img_left(self, data):
        """Saves the last left image message."""
        self.img_left = data

    def callback_img_right(self, data):
        """Saves the last right image message."""
        self.img_right = data

    def call_recognizer(self, img, side):
        """Sends the last img to the toy recognize server and publishes the result if there is enough confidence."""
        # Only if there is an img
        if img is not None:
            try:
                result = self._srv(image=img)
            except Exception as e:
                print(str(e))
                return
            # Get the best recognition
            for r in result.recognitions:
                best = CategoryProbability(label="unknown", probability=r.categorical_distribution.unknown_probability)
                for p in r.categorical_distribution.probabilities:
                        if p.probability > best.probability:
                                    best = p
                # If the confidence is higher than 80% publish the result and which camera the img came from
                if best.probability > 0.8:
                    print(best)
                    result = ObjectSide()
                    result.object = best.label
                    result.side = side
                    self.toy_pub.publish(result)


def main(args):
    """Calls the method to recognize the toys."""
    # Create the client
    trc = toy_recognizer_client()
    rospy.init_node('toy_recognizer_client', anonymous=True)
    # Send two images per second
    rate = rospy.Rate(2)
    i = 0
    try:
        while not rospy.is_shutdown():
                rate.sleep()
                # Alternate between the right and the left camera
                if i % 2 == 0:
                    img = trc.img_left
                    side = "left"
                else:
                    img = trc.img_right
                    side = "right"
                trc.call_recognizer(img, side)
                i += 1
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
        main(sys.argv)