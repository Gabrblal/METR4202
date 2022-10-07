#! /usr/bin/python3

from threading import Lock
from typing import Sequence

import rospy as ros
from tf.transformations import euler_from_quaternion

from numpy import asarray, ndarray, eye
from numpy.linalg import norm

import utility.robot as robot
from utility.topics import Topics, JointState, Header, Pose

from utility.trajectory import Spline, SCurve, Linear
from utility.kinematics import inverse_kinematics, newton_raphson

class CarouselTrajectory:
    """Node responsible for handling the trajectory of the end effector by
    taking the current desired end effector configuratio

    Class responsible for calculating trajectories and pose of the carousel
    arm.
    """

    def __init__(
            self,
            screws : Sequence[ndarray],
            M : ndarray,
            link_lengths : Sequence[float],
            joint_names : Sequence[str] = ('joint_1', 'joint_2', 'joint_3', 'joint_4'),
            method = 'analytical',
            rate_max : int  = 100,
            threshold  : int = 10
        ):
        """Create a new carousel poser.

        Args:
            screws: The screws of each joint.
            M: The home configuration of the end effector.
            link_lengths: The lengths of the links to the effector.
            joint_names: A sequence of strings naming each joint.
            method: The method to use for inverse kinematics. One of
                'analytical', 'numerical' or 'random'.
            rate_max: The maximum publishing rate of joint varaibles.
            threshold: The threshold between the old and new desired end
                effector configuration before regenerating the trajectory.
        """
        self._screws = screws
        self._M = M
        self._link_lengths = link_lengths
        self._joint_names = joint_names

        # Inputs are the desired end effector location and the locations of
        # the fiducial boxes.
        self._effector_sub = Topics.effector.subscriber(self._effector_callback)
        self._desired_sub = Topics.effector_desired.subscriber(self._desired_callback)
        self._box_sub = Topics.box.subscriber(self._box_callback)

        # Output is the positions of all the joints.
        self._trajectory = None
        self._joint_pub = Topics.desired_joint_states.publisher()
        self._rate = ros.Rate(rate_max)

        # The initial desired orientation is the identity rotation extended
        # vertically to almost the maximum.
        self._threshold = threshold

        # The desired end effector configuration (R, p).
        self._desired = None

        # The current end effector configuration (R, p).
        self._current = None

        # Lock protecting concurrent data access from callbacks.
        self._lock = Lock()

        # The last theta angle sent.
        self._last = None

        if method == 'numerical':
            self._inverse = self._inverse_numerical
        elif method == 'random':
            self._inverse = self._inverse_random
        else:
            self._inverse = self._inverse_analytical

    def _regenerate_trajectory(self, Rp0, Rp1):
        """Regenerates the trajectory from an initial configuration to a
        final configuration.

        Args:
            Rp0: A tuple of the rotation and translation (R, p) of the initial
                configuration.
            Rp1: A tuple of the rotation and translation (R, p) of the final
                configuration.
        """
        R0, p0 = Rp0
        R1, p1 = Rp1

        # Vertical offset proportional to the distance travelled along the
        # trajectory.
        offset = asarray([0, 0, norm(p0 - p1) / 5])

        now = ros.gettime()
        self._trajectory = Spline(
            [
                p0,
                p0 + offset ,
                p1 + offset ,
                p1
            ],
            Linear(now, now + norm(p0 - p1) / 40)
            # SCurve.from_velocity_acceleration(now, 40 / 100, (40 / 100) ** 2)
        )

    def _effector_callback(self, message : Pose):
        """Callback for when a true end effector configuration is published."""
        self._lock.acquire()

        if not isinstance(message, Pose):
            ros.logwarn("CarouselTrajectory end effector not Pose.")
            self._lock.release()
            return

        # If a desired end effector configuration hsa been received before
        # the end effector location is known, regenerate the trajectory.
        regenerate = self._current is None and self._desired is not None

        q = message.orientation
        self._current = (
            euler_from_quaternion([q.x, q.y, q.z, q.w]),
            asarray(message.position)
        )

        if regenerate:
            self._regenerate_trajectory(self._current, self._desired)

        self._lock.release()

    def _desired_callback(self, message : Pose):
        """Callback for when a new desired end effector configuration is
        published.
        """
        self._lock.acquire()

        if not isinstance(message, Pose):
            ros.logwarn("CarouselTrajectory desired effector not Pose.")
            self._lock.release()
            return

        q = message.orientation
        new = (
            euler_from_quaternion([q.x, q.y, q.z, q.w]),
            asarray(message.position)
        )

        if self._desired is None:
            self._desired = new

        if not self._current:
            self._lock.release()
            return

        # Get the hold desired position of the end effector.
        old = self._desired[1]
        current = self._current[1]

        # If the change in desired position has reached the threshold, then
        # recalculate the trajectory.
        if not norm(new - old) < self._threshold:
            now = ros.gettime()
            self._trajectory = Spline(
                [current, current, new, new],
                Linear(now, now + norm(new - old) / 40)
                # SCurve.from_velocity_acceleration(now, 40 / 100, (40 / 100) ** 2)
            )

        self._lock.release()

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

    def _inverse_random(self, R : ndarray, p : ndarray):
        """Get random theta values.

        Args:
            R: The rotation of the end effector.
            p: The position of the end effector.

        Returns:
            The joint parameters of the robot joints.
        """
        from random import uniform
        ros.sleep(0.5)
        return [uniform(-1.5, 1.5) for _ in self._joint_names]

    def _inverse_numerical(self, R : ndarray, p : ndarray):
        """Get the analytical inverse kinematics.

        Args:
            R: The rotation of the end effector.
            p: The position of the end effector.

        Returns:
            The joint parameters of the robot joints.
        """
        T, T[:3, :3], T[:3, 3] = eye(4), R, p
        return newton_raphson(
            self.robot.screws,
            self.self.robot.M,
            T,
            self._last
        )

    def _inverse_analytical(self, R : ndarray, p : ndarray):
        """Get the analytical inverse kinematics.

        Args:
            R: The rotation of the end effector.
            p: The position of the end effector.

        Returns:
            The joint parameters of the robot joints.
        """
        return inverse_kinematics(p, self._link_lengths)

    def main(self):
        """The main trajectory loop."""

        # Wait for the first trajectory to be generated.
        while not ros.is_shutdown():

            ready = False
            self._lock.acquire()
            ready = self._trajectory is not None and self._current is not None
            self._lock.release()

            if ready:
                break

            self._rate.sleep()

        # Send joint states of the current trajectory.
        while not ros.is_shutdown():

            self._lock.acquire()
            p = self._trajectory.translation(ros.get_time())
            self._lock.release()

            print(p)

            theta = self._inverse(None, p)

            # If theta was not able to be determined then don't do anything.
            if theta is not None:
                self._send_joint_states(theta)
                pass

            self._rate.sleep()

def main():
    ros.init_node('carouselTrajectory')
    carousel = CarouselTrajectory(
        robot.carousel.screws,
        robot.carousel.M,
        robot.carousel.L,
        method = 'analytical'
    )
    carousel.main()

if __name__ == '__main__':
    main()
