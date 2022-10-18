#! /usr/bin/env python

import imghdr
import rospy
import cv2

from utility.topics import Topics
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Colour:

    def __init__(self):
        self.bridge = CvBridge()
        self._colour_pub = Topics.block_colour.publisher()
        self._colour_sub = Topics.colour_info.subscriber(self._callback)

    def _callback(self, data):
        global img
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        # get rgb values
        bgr = img[img.shape[0] // 2, img.shape[1] // 2, :]

        r = bgr[2]
        g = bgr[1]
        b = bgr[0]

        print(f'red: {r} \n green: {g} \n blue: {b}')

    def main(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('colour_node', anonymous=True)
    Colour().main()
