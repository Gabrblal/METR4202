#!/usr/bin/python

import rospy
import tf
import geometry_msgs.msg
from std_msgs.msg import String

if __name__ == "__main__":
    rospy.init_node('fiducial_listener')
    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/fiducial_22', 'ximea', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print(f'transform: {trans}, rotation: {rot}')
        rate.sleep()