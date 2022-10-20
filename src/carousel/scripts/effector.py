#! /usr/bin/python3

#imports if needed
from utility.kinematics import poe
import rospy as ros
from modern_robotics import FKinSpace
import numpy as np

from utility.robot import carousel
from utility.topics import Topics, JointState, Pose

if __name__ == '__main__':
    ros.init_node('CarouselEffectorNode')
    ros.loginfo("Started effector node.")

    effector_pub = Topics.effector.publisher()

    def _callBack(message : JointState):
        # put theta in form that poe wants

        name = message.name
        position = message.position

        # Ensure all the joint angles are available.
        required = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        for joint in required:
            if joint not in name:
                return

        angles = [position[name.index(j)] for j in ['joint_1', 'joint_2', 'joint_3', 'joint_4']]

        R, p = poe(carousel.M, carousel.screws, angles, decomposed = True)

        from math import degrees
        ros.loginfo(f'Forward: {[round(degrees(t), 2) for t in angles]} -> {[round(x, 2) for x in p]}.')

        pose = Pose()
        pose.position.x = p[0]
        pose.position.y = p[1]
        pose.position.z = p[2]
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1

        # ros.loginfo("Publishing end effector.")
        effector_pub.publish(pose)

    #publishes to effector, see topics.py
    joint_sub = Topics.joint_states.subscriber(_callBack)

    ros.spin()
