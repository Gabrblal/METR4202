#! /usr/bin/python3

from threading import Lock
from typing import Sequence

import rospy as ros
from tf.transformations import euler_from_quaternion

from scipy.spatial.transform import Rotation
from numpy import asarray, ndarray, eye
from numpy.linalg import norm

import utility.robot as robot
from utility.topics import Topics, JointState, Header, Pose

from utility.trajectory import Spline, SCurve, Linear
from utility.kinematics import inverse_analytical_4R, newton_raphson

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
            rate_max : int  = 100,
            threshold  : int = 0
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

        # Output is the positions of all the joints.
        self._trajectory = None
        self._rate = ros.Rate(rate_max)

        # The initial desired orientation is the identity rotation extended
        # vertically to almost the maximum.
        self._threshold = threshold

        # The desired end effector configuration (R, p).
        self._desired = None
        self._desired_pitch = None

        # The current end effector configuration (R, p).
        self._current = None

        # Lock protecting concurrent data access from callbacks.
        self._lock = Lock()

        # The last theta angle sent.
        self._last = None

        # Inputs are the desired end effector location and the locations of
        # the fiducial boxes.
        self._effector_sub = Topics.effector.subscriber(self._effector_callback)
        self._desired_sub = Topics.effector_desired.subscriber(self._desired_callback)
        self._joint_pub = Topics.desired_joint_states.publisher()

        ros.loginfo('Initialised trajectory node.')

    def _regenerate_trajectory(self, Rp0, Rp1):
        """Regenerates the trajectory from an initial configuration to a
        final configuration.

        Args:
            Rp0: A tuple of the rotation and translation (R, p) of the initial
                configuration.
            Rp1: A tuple of the rotation and translation (R, p) of the final
                configuration.
        """
        ros.loginfo('Regenerating Trajectory!')

        pitch0, p0 = Rp0
        pitch1, p1 = Rp1

        # Vertical offset proportional to the distance travelled along the
        # trajectory.
        vector = p1 - p0

        now = ros.get_time()
        self._trajectory = Spline(
            [
                p0,
                p0 + vector * 0.33,
                p0 + vector * 0.66,
                p1
            ],
            Linear(now, 0.1)
            # SCurve.from_velocity_acceleration(now, 40 / 100, (40 / 100) ** 2)
        )
        self._desired_pitch = pitch1

        ros.loginfo(
            f'Regeneration trajectory '
            f'{p0.flatten()} -> {p1.flatten()}'
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
        p = message.position

        _, pitch, _ = Rotation.from_quat(
            [q.x, q.y, q.z, q.w]
        ).as_euler('ZYX')

        self._current = (pitch, asarray([p.x, p.y, p.z]))

        # ros.loginfo(f'Got end effector position {self._current[1]}')

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
        p = message.position

        _, pitch, _ = Rotation.from_quat(
            [q.x, q.y, q.z, q.w]
        ).as_euler('ZYX')

        new = (pitch, asarray([p.x, p.y, p.z]))

        ros.loginfo(f'Got desired position {new[1]} at pitch {pitch}')

        if self._desired is None:
            self._desired = new

            if self._current is not None:
                self._regenerate_trajectory(self._current, new)
                self._lock.release()
                return

        if not self._current:
            ros.loginfo("Current position unknown. Not regenerating.")
            self._lock.release()
            return

        # Get the hold desired position of the end effector.
        old = self._desired

        # If the change in desired position has reached the threshold, then
        # recalculate the trajectory.
        if norm(new[1] - old[1]) > self._threshold:
            self._regenerate_trajectory(self._current, new)
        else:
            ros.loginfo(
                'Threshold not reached to regenerate trajectory. '
                f'{norm(new[1] - old[1])} < {self._threshold}'
            )

        self._lock.release()

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
                velocity = [1.0, 1.0, 1.0, 1.0]
            )
        )

    def main(self):
        """The main trajectory loop."""

        # Wait for the first trajectory to be generated.
        while not ros.is_shutdown():

            ready = False
            self._lock.acquire()
            ready = self._trajectory is not None
            self._lock.release()

            if ready:
                break

            self._rate.sleep()

        ros.loginfo('Ready!')

        # Send joint states of the current trajectory.
        while not ros.is_shutdown():

            self._lock.acquire()
            p = self._trajectory(ros.get_time())
            pitch = self._desired_pitch
            self._lock.release()

            theta = inverse_analytical_4R(p, self._link_lengths, pitch)

            # If theta was not able to be determined then don't do anything.
            self._send_joint_states(theta)

            self._rate.sleep()

def main():
    ros.init_node('CarouselTrajectoryNode')
    carousel = CarouselTrajectory(
        robot.carousel.screws,
        robot.carousel.M,
        robot.carousel.L
    )
    carousel.main()

if __name__ == '__main__':
    main()
