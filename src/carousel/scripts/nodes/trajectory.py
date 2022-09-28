import rospy as ros

from typing import Sequence

from ..topics import Topics

# https://github.com/UQ-METR4202/metr4202_w7_prac/blob/main/scripts/joint_states_publisher.py

class CarouselTrajectory:
    """Node responsible for handling the trajectory of the end effector by
    taking the current desired end effector configuratio

    Class responsible for calculating trajectories and pose of the carousel
    arm.
    """

    def __init__(
            self,
            joint_names : Sequence[str] = ('joint_1', 'joint_2', 'joint_3', 'joint_4')
        ):
        """Create a new carousel poser.

        Args:
            joints: A sequence of strings naming each joint.
        """
        self._joint_names = joint_names

        # Inputs are the desired end effector location and the locations of
        # the fiducial boxes.
        self._effector_sub = Topics.effector.subscriber(self._effector_callback)
        self._desired_sub = Topics.effector_desired.subscriber(self._desired_callback)
        self._box_sub = Topics.box.subscriber(self._box_callback)

        # Output is the positions of all the joints.
        self._joint_pub = Topics.desired_joint_states.publisher()

    def _effector_callback(self):
        """Callback for when a true end effector configuration is published."""
        pass

    def _desired_callback(self):
        """Callback for when a new desired end effector configuration is
        published.
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
        pass
        # self._desired_joint_states_pub.publish(
        #     JointState(
        #         header = Header(stamp = ros.Time.now()),
        #         name = CarouselPoser.JOINTS,
        #         position = theta
        #     )
        # )

    def main(self):
        ros.spin()

if __name__ == '__main__':
    CarouselTrajectory().main()
