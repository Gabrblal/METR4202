import rospy as ros

from typing import Collection

from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

# https://github.com/UQ-METR4202/metr4202_w7_prac/blob/main/scripts/joint_states_publisher.py

class CarouselPoser:
    """Class responsible for calculating trajectories and pose of the carousel
    arm.
    """

    JOINTS = ('joint_1', 'joint_2', 'joint_3', 'joint_4')

    def __init__(
            self,
            carousel_effector_desired : str = 'carousel_effector_desired',
            carousel_joints_desired : str = 'desired_joint_states'
        ):
        """Create a new carousel poser."""
        self._carousel_effector_desired = ros.Subscriber(carousel_effector_desired, Pose, queue_size = 10)
        self._carousel_joints_desired = ros.Publisher(carousel_joints_desired, )
        self._publisher = None
        self._rate

    def _end_effector_callback(self):
        pass

    def _send_joint_states(self, theta : Collection[float]):
        """Publishes a joint state message containing the angles of each.

        Args:
            theta: A collection (theta1, theta2, theta3, theta4) of the angles
                   of each joint on the robot.
        """
        self._carousel_joints_desired.publish(
            JointState(
                header = Header(stamp = ros.Time.now()),
                name = CarouselPoser.JOINTS,
                position = theta
            )
        )

    def main(self):
        ros.spin()

if __name__ == '__main__':
    CarosouelPoser().main()
