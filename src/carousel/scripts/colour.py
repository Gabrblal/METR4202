#! /usr/bin/env python

import rospy as ros

from utility.topics import Topics
from std_msgs.msg import String
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

        colour = String
        if r > 150 and b < 100:
            if g > 150:
                colour = "yellow"
            else:
                colour = "red"
        elif g > 150 and r < 140 and b < 100:
            colour = "green"
        elif b > 100 and r < 100:
            colour = "blue"
        else:
            colour = "no block found"

        # ros.loginfo(f'\n red: {r} \n green: {g} \n blue: {b} \n colour: {colour}')

        # colour = ColorRGBA()
        # colour.r = r
        # colour.g = g
        # colour.b = b
        self._colour_pub.publish(colour)

    def main(self):
        ros.spin()

if __name__ == "__main__":
    ros.init_node('CarouselColourNode', anonymous=True)
    Colour().main()
