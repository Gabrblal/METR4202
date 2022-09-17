#! /usr/bin/env python3

import rospy as ros
from std_msgs.msg import String

class TimePublisher:

    def __init__(self, name = "time", rate = 10):
        """Create a time publisher.

        Args:
            name: The name of the time publisher node.
            rate: The rate in hertz that the publisher should publish.
        """
        ros.init_node(name)
        self._name = name
        self._publisher = ros.Publisher(name, String, queue_size = 10)
        self._rate = ros.Rate(rate)

    def main(self):

        while not ros.is_shutdown():
            t = ros.get_time()
            self._publisher.publish(
                String(data = f"The time is {t}")
            )
            print(f"Published {t}")
            self._rate.sleep()

if __name__ == '__main__':
    try:
        TimePublisher("time").main()
    except ros.ROSInterruptException:
        pass