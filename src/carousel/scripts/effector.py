#! /usr/bin/python3

from math import degrees

import rospy as ros

from utility.kinematics import poe
from utility.robot import carousel
from utility.topics import Topics, JointState, Pose

if __name__ == '__main__':
    ros.init_node('CarouselEffectorNode')
    ros.loginfo("Started effector node.")

    effector_pub = Topics.effector.publisher()

    def _callback(message : JointState):
        """Callback on joint state message to calculate forward
        kinematics end effector location and publish the current
        end effector position.

        Args:
            message: The joint state message of the current joint angles.
        """

        name = message.name
        position = message.position

        # Ensure all the joint angles are available.
        required = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        for joint in required:
            if joint not in name:
                return

        # Get the ordered joint angles.
        angles = [position[name.index(j)] for j in ['joint_1', 'joint_2', 'joint_3', 'joint_4']]

        # Calculate the orientation and position of the end effector.
        R, p = poe(carousel.M, carousel.screws, angles, decomposed = True)

        pose = Pose()

        # End effector position.
        pose.position.x = p[0]
        pose.position.y = p[1]
        pose.position.z = p[2]

        # Identity rotation.
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1

        effector_pub.publish(pose)

    # Subscribe 
    joint_sub = Topics.joint_states.subscriber(_callback)
    ros.spin()
