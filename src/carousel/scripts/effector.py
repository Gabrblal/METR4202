#! /usr/bin/python3

#imports if needed
from carousel.scripts.utility.kinematics import poe
import rospy as ros

from utility.robot import carousel
from utility.topics import Topics, JointState, Pose

if __name__ == '__main__':

    #subscribes to joint_states, see topics.py
    effector_pub = Topics.effector.publisher()

    # use utility/kinematics/py to calculate, 
    # with screws from robot.py this is what the carousel.M and carousel.screws is
    # R, p = is a fancy python decomposing tuple, we want the p
    # the decomposed = true is giving seperated rotation and position matrix's
    def _callBack(message : JointState):
        # put theta in form that poe wants
        R, p = poe(carousel.M, carousel.screws, message.position, decomposed = True)
    
        pose = Pose()
        pose.position.x = p[0]
        pose.position.y = p[1]
        pose.position.z = p[2]
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 0

        # publish p
        effector_pub.publish(pose)

    #publishes to effector, see topics.py
    joint_sub = Topics.joint_states.subscriber(_callBack)

    ros.init_node('CarouselEndEffectorNode')
    ros.spin()

