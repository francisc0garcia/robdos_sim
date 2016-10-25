#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class robdos_camera_test:

    def __init__(self):
        self.image_frontal = np.zeros((480, 640, 3), np.uint8)
        self.image_rear = np.zeros((480, 640, 3), np.uint8)

        #create publisher for processed image
        self.image_pub = rospy.Publisher("/robdos/camera_rear_processed/image_raw", Image, queue_size=1)

        self.bridge = CvBridge()

        # Subscribe for images:
        self.image_sub = rospy.Subscriber("/robdos/camera_rear/image_raw/compressed", Image, self.callback_rear)
        self.image_sub = rospy.Subscriber("/robdos/camera_frontal/image_raw", Image, self.callback_frontal)

    def callback_frontal(self,data):
        try:
            self.image_frontal = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.process_images()
        except CvBridgeError as e:
            print(e)

    def callback_rear(self,data):
        try:
            self.image_rear = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.process_images()
        except CvBridgeError as e:
            print(e)

    def process_images(self):
        image_1 = self.image_frontal
        image_2 = self.image_rear

        ''' some image processing using opencv '''
        image_2 = cv2.Canny(image_2, 50, 100)
        image_2 = cv2.cvtColor(image_2,cv2.COLOR_GRAY2RGB)

        both = np.hstack((image_1, image_2))

        #Republish processed image:
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(both, "bgr8"))


def main(args):
    ic = robdos_camera_test()
    rospy.init_node('robdos_camera_test', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)