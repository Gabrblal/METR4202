#! /usr/bin/env python

import rospy
from utility.topics import Topics


class Colour:

    def __init__(self):
        self._colour_pub = Topics.luggage_colour.publisher()
        self._colour_sub = Topics.colour_get.subscriber(self._callback)
        pass

    def _callback(self, data):
        # Do stuff here
        pass

    def main(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('colour_finder', anonymous=True)
    Colour().main()
