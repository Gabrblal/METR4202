#! /usr/bin/python3

import rospy as ros
from tf.transformations import euler_matrix

from numpy import asarray
from typing import Sequence

from utility.topics import Topics, JointState, Header
from utility.trajectory import Trajectory
from utility.kinematics import newton_raphson

# https://github.com/UQ-METR4202/metr4202_w7_prac/blob/main/scripts/joint_states_publisher.py

class CarouselTrajectory:
    """Node responsible for handling the trajectory of the end effector by
    taking the current desired end effector configuratio

    Class responsible for calculating trajectories and pose of the carousel
    arm.
    """

    def __init__(
            self,
            screws,
            M,
            joint_names : Sequence[str] = ('joint_1', 'joint_2', 'joint_3', 'joint_4')
        ):
        """Create a new carousel poser.

        Args:
            screws: The screws of each joint.
            M: The home configuration of the end effector.
            joints: A sequence of strings naming each joint.
        """
        self._screws = screws
        self._M = M
        self._joint_names = joint_names

        # Inputs are the desired end effector location and the locations of
        # the fiducial boxes.
        self._effector_sub = Topics.effector.subscriber(self._effector_callback)
        self._desired_sub = Topics.effector_desired.subscriber(self._desired_callback)
        self._box_sub = Topics.box.subscriber(self._box_callback)

        # Output is the positions of all the joints.
        self._trajectory = None
        self._joint_pub = Topics.desired_joint_states.publisher()


    def _effector_callback(self):
        """Callback for when a true end effector configuration is published."""
        pass

    def _desired_callback(self):
        """Callback for when a new desired end effector configuration is
        published.`
        """
        pass

    def _box_callback(self):
        """Callback for when a new cube configuration is published."""
        pass

    def _send_joint_states(self, theta : Sequence[float]):
        """Publishes a joint state message containing the angles of each joint.

        Args:
            theta: A sequence of joint parameters for each joint on the arm.
        """
        self._joint_pub.publish(
            JointState(
                header = Header(stamp = ros.Time.now()),
                name = self._joint_names,
                position = theta,
                velocity = [0.5, 0.5, 0.5, 0.5]
            )
        )

    def _inverse_kinematics(self, theta, guess = None):
        if guess is None:
            guess = asarray([0 for _ in theta])

        return newton_raphson(
            self._screws,
            self._M,
            theta,
            guess
        )

    def main(self):

        from math import atan2, asin, degrees

        x0 = -100
        x1 = 100
        y = 200
        z = 200
        yaw0 = atan2(y, x0)
        yaw1 = atan2(y, x1)

        start = ros.get_time()
        self._trajectory = Trajectory(
            [
                (-100, y, z),
                (-50, y, z),
                (50, y, z),
                (100, y, z)
            ],
            [euler_matrix(0, 0, yaw0)[:3, :3],  euler_matrix(0, 0, yaw1)[:3, :3]],
            start,
            50,
            10
        )

        from numpy import linspace
        print("Trajectory:")
        for t in linspace(start, start + self._trajectory.duration, 10):
            T = self._trajectory.at(t)
            print(f"angle: {degrees(asin(T[1][0]))}")
            # print(T)

        # from random import uniform
        # theta = [uniform(-1.5, 1.5) for _ in self._joint_names]

        theta_last = None
        while not ros.is_shutdown():
            desired = self._trajectory.at(ros.get_time())
            theta = self._inverse_kinematics(desired, theta_last)
    
            if theta is not None:
                theta_last = theta
                print(theta)
                # self._send_joint_states(theta)

def main():
    # from numpy import concatenate, cross
    # screw = lambda w, p: concatenate((w, -cross(w, p)))
    # screws = asarray([
    #     screw(asarray(w), asarray(p)) for w, p in (
    #         ((0, 0, 1), (0, 0, 0)),
    #         ((1, 0, 0), (0, 0, 54)),
    #         ((1, 0, 0), (0, 0, 171)),
    #         ((1, 0, 0), (0, 0, 265))
    #     )
    # ])

    ros.init_node('carouselTrajectory')

    screws = asarray([
       [  0,   0,   1,   0,   0,   0],
       [  1,   0,   0,   0,  54,   0],
       [  1,   0,   0,   0, 171,   0],
       [  1,   0,   0,   0, 265,   0]
    ])
    M = asarray([
        [1, 0, 0, 0],
        [0, 0, -1, 12],
        [0, 1, 0, 303],
        [0, 0, 0, 1],
    ])

    carousel = CarouselTrajectory(screws, M)
    carousel.main()

if __name__ == '__main__':
    main()
