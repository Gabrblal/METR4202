import rospy as ros

from dataclasses import dataclass
from enum import Enum
from typing import Callable, Any

from std_msgs.msg import Float32, ColorRGBA, Header, String, Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState, Image
from fiducial_msgs.msg import FiducialTransformArray

@dataclass
class Topic:
    """A general topic used by the carousel."""
    string : str = None
    type : Any = None

    def subscriber(self, callback : Callable):
        """Create and return a subscriber to this topic.

        Args:
            callback: The callback to call when receiving a message on this
                      topic.

        Returns:
            The subscriber to this topic.
        """
        return ros.Subscriber(self.string, self.type, callback)

    def publisher(self, *, queue_size = 10):
        """Create and return a publisher to this topic.

        Args:
            queue_size: The size of the message queue.

        Returns:
            A publisher to this topic.FiducialTransformArray)
        """
        return ros.Publisher(self.string, self.type, queue_size = queue_size)

class Topics:
    """A collection of topics used for communication between nodes in the
    carousel code."""

    # Topic about the fiducial boxes, their identifiers, orientation and
    # positions.
    box = Topic('carousel_box', Pose)

    # Topic about the maximum colour observed by the camera.
    # max_colour = Topic('carousel_colour', ColorRGBA)

    # Topic about the real life end effector configuration.
    effector = Topic('carousel_effector', Pose)

    # Topic about the desired configuration of the end effector.
    effector_desired = Topic('carousel_effector_desired', Pose)

    # Topic about the current gripper width.
    gripper = Topic('carousel_gripper', Float32)

    # Topic about the desired joint states provided by the dynamixel library.
    desired_joint_states = Topic('desired_joint_states', JointState)

    # Topic about the joint states provided by the dynamixel library.
    joint_states = Topic('joint_states', JointState)

    # Topic about the fiducial information provided by the aruco library
    fiducial_transforms = Topic('fiducial_transforms', FiducialTransformArray)

    # Topic about the corrected frame transforms to the home frame {0}.
    correct_frames = Topic('correct_frame', FiducialTransformArray)

    # Topic about the colour of the block.
    # block_colour = Topic('block_colour', ColorRGBA)
    block_colour = Topic('block_colour', String)

    # Topic to enable or disable rbg mode.
    rgb_mode = Topic('ximea_ros/show_rgb', Bool)

    # Topic about image colour that camera is reading.
    serial = 31701651
    colour_info = Topic(f'ximea_ros/ximea_{serial}/image_raw', Image)
