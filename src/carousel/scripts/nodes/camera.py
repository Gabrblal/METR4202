#!/usr/bin/python

import rospy
import tf
import geometry_msgs.msg
from std_msgs.msg import String
from fiducial_msgs.msg import FiducialTransformArray

if __name__ == "__main__":
    rospy.init_node('aruco_frames')
    listener = tf.TransformListener()
    sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # rospy.loginfo('Node has started')
        # camera_frame_pub = rospy.Publisher('camera/frame')
        # try:
        #     (trans, rot) = listener.lookupTransform('/fiducial_10', 'ximea', rospy.Time(0))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     continue
        # print(f'transform: {trans}, rotation: {rot}')
        rate.sleep()